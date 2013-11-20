#include <stdio.h>
#include <avr/io.h>
#include <util/setbaud.h>

#define BAUD 57600

extern void uart_init (void);
extern void uart_putchar (char c, FILE *stream);

extern FILE uart_output;
