#include "heater.h"
#include "pin.h"

void heater_on(void)
{
  if (pin_read(37) == 0)
    pin_high(37);
}

void heater_off(void)
{
  if (pin_read(37) == 1)
    pin_low(37);
}
