#include "ir.h"

// CS12=0 CS11=1 CS10=0 => Set prescaler to clock/8
// 16MHz clock with prescaler means TCNT1 increments every 0.5us
//
// This yields a max possible timer period of ~32.77ms; however, we are
// effectively limited by MAX_PULSE
#define timer1_on  { TCCR1B |= (1 << CS11); }
#define timer1_off { TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); }

// double buffering
static volatile uint8_t ir_raw_a[IR_RAW_SIZE];
static volatile uint8_t ir_raw_b[IR_RAW_SIZE];
static volatile uint8_t *ir_signal = ir_raw_a;
volatile uint8_t *ir_signal_readcopy = ir_raw_b;

static volatile uint8_t change_count = 0;
static volatile bool receiving = 0;
static volatile bool data_available = 0;

static uint8_t ir_commands[NUMBER_OF_IR_CODES][IR_RAW_SIZE];

const char *command_labels[NUMBER_OF_IR_CODES] = {
  "Volume Up",
  "Volume Down",
  "Mute"
};

static void ir_reset (void);
static void flip_buffers (void);

void ir_init (void) {
  DDRD  &= ~(1 << PD2);     // PD2/D2/Infrared as input
  PORTD |=  (1 << PD2);     // pull-up on

  EICRA |=  (1 << ISC00);   // trigger INT0 on any change

  // Timer1 setup
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12);
  TCNT1  = 0x0000;
  OCR1A  = (MAX_PULSE - 1); // 10ms
  TIMSK1 = (1 << OCIE1A);
}

void ir_enable (void) {
  EIMSK |=  (1 << INT0);
}

void ir_disable (void) {
  EIMSK &=  ~(1 << INT0);
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
  tmp = ir_signal_readcopy;
  ir_signal_readcopy = ir_signal;
  ir_signal = tmp;
}
