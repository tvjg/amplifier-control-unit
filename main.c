#define BAUD 57600

#include <stdio.h>
#include <util/setbaud.h>

#include "ir.h"

#define SERIAL_LOG
#define IR_DEBUG_RAW

void uart_init(void);
void uart_putchar(char c, FILE *stream);
void dump_ir_raw(void);

int main (void) {

#ifdef SERIAL_LOG
  uart_init();

  FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uart_output;

  // TODO: If space gets tight, consider moving strings to flash memory.
  // http://playground.arduino.cc/Main/Printf#sourceblock5
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
      ir_command_t command = match_ir_code(ir_signal_readcopy);
      const char* msg = (command < 0) ? "Unknown Code" : command_labels[command];

#ifdef SERIAL_LOG
#ifdef IR_DEBUG_RAW
      dump_ir_raw();
#endif

      printf("Diagnosis: %s\n\n", msg);
#endif
    }
  }
}

void uart_init (void) {
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX
}

void uart_putchar(char c, FILE *stream) {
  if (c == '\n') {
    uart_putchar('\r', stream);
  }
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
}

void dump_ir_raw (void) {
  printf("Received: \nOFF\t\tON\n");
  uint8_t i;
  for (i = 0; i < IR_RAW_SIZE; i+=2) {
    printf("%d\t\t%d\n", ir_signal_readcopy[i], ir_signal_readcopy[i+1]);

    if (ir_signal_readcopy[i+1] == 0xFF) break;
  }

  printf("\nPulse Count: %d\n", i+2);
}
