#ifndef __PIN_H
#define __PIN_H

int pin_high(int pin);
int pin_float(int pin);
int pin_low(int pin);
int pin_input(int pin, int pullup);
int pin_output(int pin);

#endif
