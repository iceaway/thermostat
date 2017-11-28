#include "cooling.h"
#include "pin.h"

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#define COOLING_PIN  D3
#elif defined (__AVR_ATmega2560__)
#define COOLING_PIN  D38
#else
#error Unsupported MCU
#endif

void cooling_init_gpio(void)
{
  pin_output(COOLING_PIN);
  pin_low(COOLING_PIN);
}

void cooling_on(void)
{
  if (pin_read(COOLING_PIN) == 0)
    pin_high(COOLING_PIN);
}

void cooling_off(void)
{
  if (pin_read(COOLING_PIN) == 1)
    pin_low(COOLING_PIN);
}
