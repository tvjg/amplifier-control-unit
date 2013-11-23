#include <stdio.h>
#include <avr/eeprom.h>

#include "uart.h"
#include "ir.h"

#define SERIAL_LOG
#define IR_LOG_VERBOSE

// TODO: Learning routine should eventually be driven by hitting a reset switch
// inside the case. For now, a flag should be set.
/*#define RESET_COMMANDS*/

// TODO: If space gets tight, consider moving strings to flash memory.
// http://playground.arduino.cc/Main/Printf#sourceblock5

void print_ir_timings (volatile uint8_t *);

int main (void) {

#ifdef SERIAL_LOG
  uart_init();
  stdout = &uart_output;

  printf("Ready to decode IR!\n\n");
#endif

  ir_init();
  sei();

#ifdef RESET_COMMANDS

  for (uint8_t i = 0; i < NUMBER_OF_IR_CODES; i++) {
    // TODO: Don't just dump to stdout willy-nilly. Will probably need
    // something for LCD.
    printf("Ready to learn %s command.\n\n", command_labels[i]);
    learn_ir_code(i);

#ifdef SERIAL_LOG
#ifdef IR_LOG_VERBOSE
    print_ir_timings(ir_signal_readcopy);
    printf("\n");
#endif

    printf("Learned %s command.\n\n", command_labels[i]);
#endif
  }

  eeprom_write_block(&ir_commands, 0x00, NUMBER_OF_IR_CODES * IR_RAW_SIZE);
#ifdef SERIAL_LOG
  printf("Commands written to EEPROM.\n\n");
#endif

#else

  eeprom_read_block(&ir_commands, 0x00, NUMBER_OF_IR_CODES * IR_RAW_SIZE);
#ifdef SERIAL_LOG
  printf("Commands read from EEPROM.\n\n");
#endif

#endif

  ir_enable();

  while (1) {
    if (ir_available()) {
      ir_command_t cmd = match_ir_command(ir_signal_readcopy);
      const char* msg = (cmd < 0) ? "Unknown" : command_labels[cmd];

#ifdef SERIAL_LOG
#ifdef IR_LOG_VERBOSE
      print_ir_timings(ir_signal_readcopy);
#endif

      printf("Command: %s\n\n", msg);
#endif
    }
  }
}

void print_ir_timings (volatile uint8_t *signal) {
  printf("Received: \nOFF\t\tON\n");
  uint8_t i;
  for (i = 0; i < IR_RAW_SIZE; i+=2) {
    printf("%d\t\t%d\n", signal[i], signal[i+1]);

    if (signal[i+1] == 0xFF) break;
  }

  printf("\nPulse Count: %d\n", i+2);
}
