#define MAXPULSE 65000U
#define MINPULSE 5U
#define NUMPULSES 150U
#define FUZZINESS 20U

// Since this receiver uses a universal learning approach, the EEPROM size sets
// the upper bound on the number of codes we can store. There are available 512
// bytes on the Atmega168, 1024 bytes on the 328. 
//
// RequiredBytes = NUMPULSES * NUMBER_OF_IR_CODES * (sizeof uint16_t).
//
// Currently:
//   432 bytes = 72 * 3 * 2 bytes
#define NUMBER_OF_IR_CODES 3

typedef enum { 
  VOL_UP,
  VOL_DOWN, 
  MUTE,
  NO_MATCH = -1
} IR_code_t;

const char *command_labels[NUMBER_OF_IR_CODES] = {
  "Volume Up",
  "Volume Down",
  "Mute"
};

