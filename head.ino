/* vim: set filetype=c : */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "head.h"

#define ext_int_on(interrupt)  { EIMSK |=  (1 << interrupt); }
#define ext_int_off(interrupt) { EIMSK &=  ~(1 << interrupt); }

// CS12=0 CS11=1 CS10=0 => Set prescaler to clock/8
// 16MHz clock with prescaler means TCNT1 increments every 0.5uS
#define timer1_on  { TCCR1B |= (1 << CS11); }
#define timer1_off { TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); }

#define   IR_PIN         PD2
#define   IR_PORT        PORTD
#define   IR_INTERRUPT   INT0

// double buffering
static volatile uint8_t ir_raw_a[IR_RAW_SIZE];
static volatile uint8_t ir_raw_b[IR_RAW_SIZE];
static volatile uint8_t *ir_signal = ir_raw_a;
static volatile uint8_t *ir_signal_copy = ir_raw_b;

static volatile uint8_t change_count = 0;
static volatile bool receiving = 0;
static volatile bool data_available = 0;

static uint8_t ir_commands[NUMBER_OF_IR_CODES][IR_RAW_SIZE];

int main (void) {
  init();  // take care of Arduino timers etc. etc.

  Serial.begin(9600);

  // F() macro neatly wraps string storage/retrieval from flash memory
  // http://playground.arduino.cc/Main/Printf#sourceblock5
  Serial.println("Ready to decode IR!");

  DDRB  |=  (1 << PB5);    // PB5/D13/LED as output

  ir_init();

  sei();

  // TODO: We need to setup a way to learn the code the codes before we enter
  // the permanent matching loop. Learning routine should eventually be driven
  // by hitting a reset switch inside the case. For now, learning should be
  // activated if no codes are found in memory or a flag is set.

  while (1) {
    if (ir_available()) {
      ir_command_t command = match_ir_code(ir_signal_copy);
      const char* msg = (command < 0) ? "Unknown Code" : command_labels[command];

      Serial.println("\n\rReceived: \n\rOFF\t\tON");
      for (uint8_t i = 0; i < change_count; i+=2) {
        Serial.print(ir_signal_copy[i], DEC);
        Serial.print("\t\t");
        Serial.println(ir_signal_copy[i+1], DEC);
      }

      Serial.print("\n\rPulse Count: ");
      Serial.println(change_count+1);
      Serial.print("Diagnosis: ");
      Serial.println(msg);
    }
  }
}

void ir_init (void) {
  DDRD  &= ~(1 << PD2);     // PD2/D2/Infrared as input
  PORTD |=  (1 << PD2);     // pull-up on

  ext_int_on(IR_INTERRUPT);
  EICRA |=  (1 << ISC00);   // trigger INT0 on any change

  // Timer1 setup
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12);
  TCNT1  = 0x0000;
  OCR1A  = (MAX_PULSE - 1); // 10ms
  TIMSK1 = (1 << OCIE1A);
}

void ir_reset (void) {
  timer1_off;
  receiving = 0;
}

bool ir_available (void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (data_available) {
      data_available = 0;
      return 1;
    }
    return 0;
  }
}

ir_command_t match_ir_code(volatile uint8_t *measured_ir_signal)
{
  return NO_MATCH;
}

ISR(INT0_vect) {
  // toggle pin 13 - led
  PORTB ^= (1 << PB5);

  if (!receiving) {
    change_count = 0;
    receiving = 1;
    TCNT1 = 0;
    timer1_on;
    return;
  }
  
  uint16_t pulse_length = TCNT1;
  
  // got some bouncing ? ignore that.
  if (pulse_length < MIN_PULSE) {
    return;                                     
  }

  ir_signal[change_count] = TCNT1H;
  change_count++;
  
  if (change_count == (IR_RAW_SIZE - 1)) {
    change_count = 0;
  }

  TCNT1 = 0;
}

ISR(TIMER1_COMPA_vect) {
  ir_signal[change_count] = 0xFF;
  
  flip_buffers();
  data_available = 1;
  
  ir_reset();
}

void flip_buffers (void) {
  volatile uint8_t *tmp;
  tmp = ir_signal_copy;
  ir_signal_copy = ir_signal;
  ir_signal = tmp;
}
