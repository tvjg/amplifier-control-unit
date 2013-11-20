/* vim: set filetype=c : */

#include "ir.h"

#define SERIAL_LOG
#define IR_DEBUG_RAW

int main (void) {
  init();  // take care of Arduino timers etc. etc.

#ifdef SERIAL_LOG
  uart_init();
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

      Serial.print("\n\rDiagnosis: ");
      Serial.println(msg);
#endif
    }
  }
}

void uart_init (void) {
  Serial.begin(57600);

  // F() macro neatly wraps string storage/retrieval from flash memory
  // http://playground.arduino.cc/Main/Printf#sourceblock5
  Serial.println("Ready to decode IR!");
}

void dump_ir_raw (void) {
  Serial.println("\n\rReceived: \n\rOFF\t\tON");
  uint8_t i = 0;
  for (i; i < IR_RAW_SIZE; i+=2) {
    Serial.print(ir_signal_readcopy[i], DEC);
    Serial.print("\t\t");
    Serial.println(ir_signal_readcopy[i+1], DEC);

    if (ir_signal_readcopy[i+1] == 0xFF) break;
  }

  Serial.print("\n\rPulse Count: ");
  Serial.println(i+2);
}
