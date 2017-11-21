#ifndef __PIN_H
#define __PIN_H


#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
enum iopins {
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  D0,
  D1,
  D2,
  D3,
  D4,
  D5,
  D6,
  D7,
  D8,
  D9,
  D10,
  D11,
  D12,
  D13
};
#elif defined (__AVR_ATmega2560__)
#else
#error Unsupported MCU
#endif

int pin_high(int pin);
int pin_float(int pin);
int pin_low(int pin);
int pin_input(int pin, int pullup);
int pin_output(int pin);
int pin_read(int pin);

#endif
