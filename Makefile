BIN=main
OBJS  = main.o rbuf.o ctrl.o env.o adc.o temperature.o pin.o heater.o 
OBJS += sched.o prints.o cooling.o

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328 -Wall -Wextra -Wl,-Map,main.map -pedantic
#CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega2560 -Wall -Wextra -Wl,-Map,main.map
#PORT=/dev/ttyACM0
PORT=/dev/ttyUSB0

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	${CC} -o $@ $^ ${CFLAGS}

install: ${BIN}.hex
  # The following line is for original mega2560
	#avrdude -v -q -D -p atmega2560 -P ${PORT} -c stk500 -b 115200 -U flash:w:$<
	#avrdude -v -q -D -p atmega328 -P ${PORT} -c stk500 -b 115200 -U flash:w:$<
	#avrdude -v -q -D -F -p atmega328 -P ${PORT} -c arduino -b 115200 -U flash:w:$<
  # The following line is for china version of arduino uno
	avrdude -v -q -D -F -p m328p -P ${PORT} -c arduino -b 115200 -U flash:w:$<
# The following line for china version with ICSP
	#avrdude -v -q -D -F -p m328p -c usbtiny -U flash:w:$<

clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
