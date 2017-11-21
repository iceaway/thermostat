#include "heater.h"
#include "pin.h"

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#define HEATER_PIN  D2
#elif defined (__AVR_ATmega2560__)
#define HEATER_PIN  D37
#else
#error Unsupported MCU
#endif

void heater_init_gpio(void)
{
  pin_output(HEATER_PIN);
  pin_low(HEATER_PIN);
}

void heater_on(void)
{
  if (pin_read(HEATER_PIN) == 0)
    pin_high(HEATER_PIN);
}

void heater_off(void)
{
  if (pin_read(HEATER_PIN) == 1)
    pin_low(HEATER_PIN);
}
