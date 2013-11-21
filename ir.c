#include "ir.h"

// CS12=0 CS11=1 CS10=0 => Set prescaler to clock/8
// 16MHz clock with prescaler means TCNT1 increments every 0.5us
//
// This yields a max possible timer period of ~32.77ms; however, we are
// effectively limited by MAX_PULSE here
#define timer1_on  { TCCR1B |= _BV(CS11); }
#define timer1_off { TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); }

// double buffering
static volatile uint8_t ir_raw_a[IR_RAW_SIZE];
static volatile uint8_t ir_raw_b[IR_RAW_SIZE];
static volatile uint8_t *ir_signal = ir_raw_a;
volatile uint8_t *ir_signal_readcopy = ir_raw_b;

static volatile uint8_t change_count = 0;
static volatile uint8_t receiving = 0;
static volatile uint8_t data_available = 0;

static uint8_t ir_commands[NUMBER_OF_IR_CODES][IR_RAW_SIZE] = {{ 0xFF }};

const char *command_labels[NUMBER_OF_IR_CODES] = {
  "Volume Up",
  "Volume Down",
  "Mute"
};

static void ir_reset (void);
static void flip_buffers (void);

void ir_init (void) {
  DDRD  &= ~_BV(PD2);     // PD2/D2/Infrared as input
  PORTD |=  _BV(PD2);     // pull-up on

  EICRA |=  _BV(ISC00);   // trigger INT0 on any change

  // Timer1 setup
  TCCR1A = 0x00;
  TCCR1B = _BV(WGM12);
  TCNT1  = 0x0000;
  OCR1A  = (MAX_PULSE - 1); // 10ms
  TIMSK1 = _BV(OCIE1A);
}

void ir_enable (void) {
  // The interrupt flag can be set even though intterupt triggering is
  // disabled. Clear before re-enabling.
  EIFR = (1 << INT0);
  EIMSK |=  _BV(INT0);
}

void ir_disable (void) {
  EIMSK &=  ~_BV(INT0);
}

void ir_reset (void) {
  timer1_off;
  receiving = 0;
}

uint8_t ir_available (void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (data_available) {
      data_available = 0;
      return 1;
    }
  }
  return 0;
}

void learn_ir_code (ir_command_t cmd) {
  ir_enable();

  while (1) {
    if (!ir_available()) continue;

    ir_disable();
    memcpy(&ir_commands[cmd], (void *) ir_signal_readcopy, IR_RAW_SIZE);
    break;
  }
}

ir_command_t match_ir_code (volatile uint8_t *measured_ir_signal) {
  return NO_MATCH;
}

ISR(INT0_vect) {
  if (!receiving) {
    change_count = 0;
    receiving = 1;
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
  tmp = ir_signal_readcopy;
  ir_signal_readcopy = ir_signal;
  ir_signal = tmp;
}
