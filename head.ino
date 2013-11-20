/* vim: set filetype=c : */

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
  ir_enable();

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

      ir_enable();
    }
  }
}

void ir_init (void) {
  DDRD  &= ~(1 << PD2);     // PD2/D2/Infrared as input
  PORTD |=  (1 << PD2);     // pull-up on

  EICRA |=  (1 << ISC00);   // trigger INT0 on any change

  // Timer1 setup
  timer1_off;
  TCCR1A = 0x00;    // Timer1 Normal mode
  TIMSK1 = 0x01;    // Timer1 Overflow Interrupt Enable
  TCNT1  = 0;

  ir_reset();
}

void ir_enable (void) {
  ir_reset();
  ext_int_on(IR_INTERRUPT);
}

void ir_disable (void) {
  ext_int_off(IR_INTERRUPT);
  timer1_off;
}

void ir_reset (void) {
  zero_pulses(ir_signal);

  receiving = 0;
}

bool ir_available (void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (data_available) {
      data_available = 0;
      return 1;
    }
  }
  return 0;
}

ir_command_t match_ir_code(volatile uint8_t *measured_ir_signal)
{
  return NO_MATCH;
}

ISR(INT0_vect) {
  // toggle pin 13 - led
  PORTB ^= (1 << PB5);

  if (!receiving) {
    timer1_on;
    change_count = 0;
    receiving = 1;
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

ISR(TIMER1_OVF_vect) {
  // wait for the caller to re-enable reception
  ir_disable();

  ir_signal[change_count] = 0xff;
  flip_buffers();

  ir_reset();
}

void zero_pulses (volatile uint8_t *array) {
  uint8_t ctr;
  for (ctr = 0; ctr < IR_RAW_SIZE; ctr++) {
    array[ctr] = 0;
  }
}

void flip_buffers (void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    zero_pulses(ir_signal_copy);

    volatile uint8_t *tmp;
    tmp = ir_signal_copy;
    ir_signal_copy = ir_signal;
    ir_signal = tmp;

    data_available = 1;
  }
}
