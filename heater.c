#include "heater.h"
#include "pin.h"

void heater_on(void)
{
  pin_high(37);
}

void heater_off(void)
{
  pin_low(37);
}
