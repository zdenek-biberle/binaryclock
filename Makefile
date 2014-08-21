PROG = clock
SOURCES = clock.c usi_i2c_master.c
HEADERS = usi_i2c_master.h
MCU1 = attiny85
MCU2 = t85

default: $(PROG).elf
	avrdude -p $(MCU2) -c usbasp-clone -U flash:w:$(PROG).elf:e

objdump: $(PROG).elf
	avr-objdump -d $(PROG).elf

$(PROG).elf: $(SOURCES) $(HEADERS)
	avr-gcc -Wall -std=c11 -Os -mmcu=$(MCU1) -o $(PROG).elf -DF_CPU=1000000ul $(SOURCES)



$.PHONY: default objdump
