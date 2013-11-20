PROJECT_NAME=head-unit
MCU=atmega168
CPU=16000000L
SOURCES=main.c uart.c ir.c
PROGRAMMER=arduino
PORT=/dev/ttyUSB0
BAUDRATE=19200

CC=avr-gcc
CFLAGS=-c -g -Os --std=gnu99 -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=$(CPU) -MMD
LDFLAGS=-Os -Wl,--gc-sections -mmcu=$(MCU) -L$(AP_PATH) -lm
OBJECTS=$(SOURCES:.c=.o)
ELFCODE=$(join $(PROJECT_NAME),.elf)
OBJ=avr-objcopy
EEPFLAGS=-O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0
HEXFLAGS=-O ihex -R .eeprom
EEPCODE=$(join $(PROJECT_NAME),.eep)
HEXCODE=$(join $(PROJECT_NAME),.hex)
AVRSIZE=avr-size
AVRSIZEFLAGS=-C --mcu=$(MCU)
AVRDUDE=avrdude
AVRDUDEFLAGS=-D -c$(PROGRAMMER) -p$(MCU) -P$(PORT) -b$(BAUDRATE)

all: clean $(OBJECTS) $(ELFCODE) $(EEPCODE) $(HEXCODE)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

$(ELFCODE):
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

$(EEPCODE):
	$(OBJ) $(EEPFLAGS) $(ELFCODE) $@

$(HEXCODE):
	$(OBJ) $(HEXFLAGS) $(ELFCODE) $@

clean:
	rm -rf $(OBJECTS) *.d $(ELFCODE) $(EEPCODE) $(HEXCODE)

size:
	$(AVRSIZE) $(AVRSIZEFLAGS) $(ELFCODE)

upload:
	$(AVRDUDE) $(AVRDUDEFLAGS) -Uflash:w:$(HEXCODE):i
