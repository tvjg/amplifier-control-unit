#include "ir.h"

// Set prescaler to clock/8
// 16MHz clock with prescaler means TCNT1 increments every 0.5us
//
// This yields a max possible timer period of ~32.77ms; however, we are
// effectively limited by MAX_PULSE here
#define timer1_on  { TCCR1B |= _BV(CS11); }
#define timer1_off { TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); }

// Set prescaler to clock/64
// 16MHz clock with prescaler means TCNT2 increments every 4us
#define timer2_on  { TCCR2B |= _BV(CS22); }
#define timer2_off { TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20)); }

// double buffering
static volatile uint8_t ir_raw_a[IR_RAW_SIZE];
static volatile uint8_t ir_raw_b[IR_RAW_SIZE];
static volatile uint8_t *ir_signal = ir_raw_a;
volatile uint8_t *ir_signal_readcopy = ir_raw_b;

static volatile uint8_t change_count = 0;
static volatile uint8_t receiving = 0;
static volatile uint8_t data_available = 0;

static struct {
  ir_command_t last_used_cmd;

  volatile uint8_t is_detected;
  volatile uint8_t is_allowed;
  volatile uint8_t _overflow_timer;
} repeat = {
  NO_MATCH, 0, 0, 0
};

static uint8_t ir_commands[NUMBER_OF_IR_CODES][IR_RAW_SIZE] = {{ 0xFF }};

const char *command_labels[NUMBER_OF_IR_CODES] = {
  "Volume Up",
  "Volume Down",
  "Mute"
};

static void ir_reset (void);
static void flip_buffers (void);
static inline uint8_t almost_match (volatile uint8_t *, volatile uint8_t *);
static inline void allow_repeats (void);

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

#ifdef HANDLE_REPEATS
  // Timer2 setup
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  TCNT2  = 0x00;
  TIMSK2 = _BV(TOIE2);
#endif
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

  // Wait to fudge for any trailing bursts like repeats
  _delay_ms(200);
}

ir_command_t match_ir_command (volatile uint8_t *signal) {

#ifdef HANDLE_REPEATS
  // This may produce incorrect corner cases if keys are mashed, but saves on
  // the expense of executing more matches
  if (repeat.is_allowed && repeat.is_detected) {
    allow_repeats();
    return repeat.last_used_cmd;
  }
#endif

  for (uint8_t i = 0; i < NUMBER_OF_IR_CODES; i++) {
    if (almost_match(signal, &ir_commands[i])) {
#ifdef HANDLE_REPEATS
      repeat.last_used_cmd = i;
      allow_repeats();
#endif

      return i;
    }
  }

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

#ifdef HANDLE_REPEATS
  repeat.is_detected = almost_match(ir_signal, ir_signal_readcopy);
#endif

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

uint8_t almost_match (volatile uint8_t *a, volatile uint8_t *b) {
  for (uint8_t t = 0; t < IR_RAW_SIZE; t++) {
    int delta = a[t] - b[t];
    uint8_t close_enough = (-2 <= delta) && (delta <= 2);

    if (!close_enough) return 0;
    if ((a[t] != 0xFF) != (b[t] != 0xFF)) return 0;

    if (a[t] == 0xFF && b[t] == 0xFF) return 1;
  }

  return 1;
}

void allow_repeats (void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    repeat.is_allowed = 1;
    repeat._overflow_timer = 0;
    TCNT2 = 0x00;
    timer2_on;
  }
}

ISR(TIMER2_OVF_vect) {
  if (++repeat._overflow_timer != 0) return;

  repeat.is_allowed = 0;
  timer2_off;
}
