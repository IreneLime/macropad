#!/bin/bash

F_CPU=16000000
MCU=atmega32u4
PROGRAM=encoder
PORT=/dev/ttyACM0

avr-gcc -mmcu=$MCU -DF_CPU=$F_CPU -o $PROGRAM.o $PROGRAM.c
avr-objcopy -O ihex -R .eeprom $PROGRAM.o $PROGRAM.hex

avrdude -p $MCU -c avr109 -P $PORT -U flash:w:$PROGRAM.hex
