#define	IR_RAW_SIZE 128
#define MIN_PULSE 5U
#define MAX_PULSE 10000U

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

const char *command_labels[NUMBER_OF_IR_CODES] = {
  "Volume Up",
  "Volume Down",
  "Mute"
};
