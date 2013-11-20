/* vim: set filetype=c : */

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "head.h"

#define SERIAL_LOG

// TODO: Do we really need volatile on the read copy?
// double buffering
static volatile uint16_t pulses_a[NUMPULSES];
static volatile uint16_t pulses_b[NUMPULSES];
static volatile uint16_t *pulses_write_to = pulses_a;
static volatile uint16_t *pulses_read_from = pulses_b;
 
static volatile uint8_t prev_pulse_count = 0;
static volatile uint8_t pulse_count = 0;

static volatile uint32_t last_IR_activity = 0;

/*static uint16_t IRsignals[NUMBER_OF_IR_CODES][NUMPULSES]; */

int main(void)
{
  init();  // take care of Arduino timers etc. etc.

  Serial.begin(9600);
  /*Serial.begin(57600);*/

  // F() macro neatly wraps string storage/retrieval from flash memory
  // http://playground.arduino.cc/Main/Printf#sourceblock5
  Serial.println("Ready to decode IR!");

  zero_pulses(pulses_read_from);
  zero_pulses(pulses_write_to);
  
  DDRB  |=  (1 << PB5);    // PB5/D13/LED as output

  DDRD  &= ~(1 << PD2);    // PD2/D2/Infrared as input
  PORTD |=  (1 << PD2);    // pull-up on

  EIMSK |=  (1 << INT0);   // enable external interrupt for pin PD2 (INT0)
  EICRA |=  (1 << ISC00);  // trigger INT0 on any change
  sei();

  // TODO: We need to setup a way to learn the code the codes before we enter
  // the permanent matching loop. Learning routine should eventually be driven
  // by hitting a reset switch inside the case. For now, learning should be
  // activated if no codes are found in memory or a flag is set.

  while (1) {
    if (IR_available()) {
      IR_code_t command = match_IR_code(pulses_read_from);
      const char* msg = (command < 0) ? "Unknown Code" : command_labels[command];

      /*Serial.println("\n\rReceived: \n\rOFF\t\t\tON");*/
      /*for (uint8_t i = 0; i < prev_pulse_count; i+=2) {*/
        /*Serial.print(pulses_read_from[i], DEC);*/
        /*Serial.print("E-5 sec");*/
        /*Serial.print("\t\t\t");*/
        /*Serial.print(pulses_read_from[i+1], DEC);*/
        /*Serial.println("E-5 sec");*/
      /*}*/

      Serial.print("\n\rPulse Count: ");
      Serial.println(prev_pulse_count);
      Serial.print("Diagnosis: ");
      Serial.println(msg);
    }
  }
}

ISR(INT0_vect)
{
  // toggle pin 13 - led
  PORTB ^= (1 << PB5);                          

  // TODO: Does abs() absolve us of any problems when timer overflows? 
  //
  // I think it would only cause issue if it occured directly in the middle of
  // IR capture. Otherwise, it will merely look like an abnormally long time
  // since the last capture.
  uint32_t now = micros();
  uint32_t pulse_length = abs(now - last_IR_activity);

  // clear the buffer after a timeout has occurred
  if (pulse_length > MAXPULSE) {
    zero_pulses(pulses_write_to);               
    pulse_count = 0;
  }
  // got some bouncing ? ignore that.
  if (pulse_length < MINPULSE) {
    return;                                     
  }

  if (pulse_count > 0) {
    pulses_write_to[pulse_count - 1] =
      (uint16_t) (pulse_length / 10);
  } else {
    // exit asap
  }

  pulse_count++;
  if (pulse_count > NUMPULSES) {
    pulse_count = 0;
  }
  last_IR_activity = micros();
}

void zero_pulses(volatile uint16_t * array)
{
  uint8_t ctr;
  for (ctr = 0; ctr < NUMPULSES; ctr++) {
    array[ctr] = 0;
  }
}

void flip_buffers(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    volatile uint16_t *tmp;
    tmp = pulses_read_from;
    pulses_read_from = pulses_write_to;
    pulses_write_to = tmp;
  }
}

uint8_t IR_available(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (last_IR_activity != 0
        && ((micros() - last_IR_activity) > MAXPULSE)) {
      flip_buffers();
      prev_pulse_count = pulse_count;
      last_IR_activity = 0;
      return 1;
    }
  }
  return 0;
}

IR_code_t match_IR_code(volatile uint16_t * pulses_measured)
{
  return NO_MATCH;
}
