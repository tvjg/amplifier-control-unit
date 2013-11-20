#include <stdio.h>
#include <avr/io.h>

// BAUD must be set first!
#define BAUD 57600
#include <util/setbaud.h>

extern void uart_init (void);
extern void uart_putchar (char c, FILE *stream);

extern FILE uart_output;
