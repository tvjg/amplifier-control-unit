#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define	IR_RAW_SIZE 128

// Length in realtime depends on prescaler settings. In this case, each timer
// tick should be 0.5us.
#define MIN_PULSE 100U    // 50us
#define MAX_PULSE 20000U  // 10000us or 10ms

// Since this receiver uses a universal learning approach, the EEPROM size sets
// the upper bound on the number of codes we can store. There are available 512
// bytes on the Atmega168, 1024 bytes on the 328.
//
// RequiredBytes = IR_RAW_SIZE * NUMBER_OF_IR_CODES * (sizeof uint8_t).
#define NUMBER_OF_IR_CODES 3

typedef enum {
  VOL_UP,
  VOL_DOWN,
  MUTE,
  NO_MATCH = -1
} ir_command_t;

extern void ir_init (void);
extern void ir_enable (void);
extern void ir_disable (void);
extern uint8_t ir_available (void);
extern void learn_ir_code (ir_command_t);
extern ir_command_t match_ir_code (volatile uint8_t *);

extern const char *command_labels[];
extern volatile uint8_t *ir_signal_readcopy;
