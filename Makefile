
.PHONY: all
all: burn

.PHONY: compile
compile:
	avr-g++ hugo.c -mmcu=atmega8 -o hugo.elf
	avr-objcopy -O ihex hugo.elf hugo.hex

.PHONY: burn
burn: compile
	avrdude -p m8 -c avr911 -P /dev/ttyUSB0 -U flash:w:hugo.hex:i 

