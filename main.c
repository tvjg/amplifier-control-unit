#include <stdio.h>

#include "uart.h"
#include "ir.h"

#define SERIAL_LOG
#define IR_LOG_VERBOSE

// TODO: We need to setup a way to learn the code the codes before we enter
// the permanent matching loop. Learning routine should eventually be driven
// by hitting a reset switch inside the case. For now, learning should be
// activated if no codes are found in memory or a flag is set.

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
  ir_enable();

  sei();

  // TODO: We need to setup a way to learn the code the codes before we enter
  // the permanent matching loop. Learning routine should eventually be driven
  // by hitting a reset switch inside the case. For now, learning should be
  // activated if no codes are found in memory or a flag is set.

  while (1) {
    if (ir_available()) {
      ir_command_t cmd = match_ir_code(ir_signal_readcopy);
      const char* msg = (cmd < 0) ? "Unknown Code" : command_labels[cmd];

#ifdef SERIAL_LOG
#ifdef IR_LOG_VERBOSE
      print_ir_timings(ir_signal_readcopy);
#endif

      printf("Diagnosis: %s\n\n", msg);
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
