#include <avr/pgmspace.h>
#include "ctrl.h"
#include "env.h"
#include "adc.h"
#include "temperature.h"
#include "heater.h"
#include "cooling.h"
#include "main.h"
#include "prints.h"

enum ctrl_state {
  OFF,
  IDLE,
  HEATING,
  COOLING
};

static int g_state = OFF;
static uint32_t g_laston = 0;

void ctrl_enable(void)
{
  g_state = IDLE;
  g_laston = get_ticks();
  prints(PSTR("Enabling ctrl loop, g_laston = %lu\r\n"), g_laston);
}

void ctrl_disable(void)
{
  g_state = OFF;
}

int ctrl_status(void)
{
  return g_state == OFF ? 0 : 1;
}

void ctrl_update(void)
{
  char tmp[16];
  float t_set;
  float t_hyst;
  uint32_t t_delay;
  float temp = temperature_get();

  if (env_get("T_SET", tmp, sizeof(tmp)) < 0) {
    return;
  }
  
  t_set = atof(tmp);
  if ((t_set < 0.0f) || (t_set > 100.0f)) {
    return;
  }

  if (env_get("T_HYST", tmp, sizeof(tmp)) < 0) {
    return;
  }
  
  t_hyst = atof(tmp);
  if (((t_set - t_hyst) < 0.0f) || ((t_set + t_hyst) > 100.0f)) {
    return;
  }

  /* Delay before allowing the cooling to activate */
  if (env_get("T_DELAY", tmp, sizeof(tmp)) < 0) {
    return;
  }
  t_delay = atoi(tmp);
  t_delay *= 1000; /* Convert to ms */

#if 0
  if (t_delay < 0)
      return;
#endif

  if (g_state != OFF)
    prints(PSTR("Temperature: %d\r\n"), (int)round(temp*10));

  switch (g_state) {
  case OFF:
    break;

  case COOLING:
    g_laston = get_ticks();
    if (temp < t_set) {
      prints(PSTR("Going to IDLE\r\n"));
      cooling_off();
      g_state = IDLE;
    }
    break;

  case HEATING:
    if (temp > t_set) {
      prints(PSTR("Going to IDLE\r\n"));
      heater_off();
      g_state = IDLE;
    }
    break;

  case IDLE:
    if (temp <= (t_set - t_hyst)) {
      prints(PSTR("Going to HEATING\r\n"));
      heater_on();
      g_state = HEATING;
    } else if (temp >= (t_set + t_hyst)) {
      uint32_t tdiff;
      tdiff = get_ticks() - g_laston;
      if (tdiff >= t_delay) {
        prints(PSTR("Going to COOLING\r\n"));
        cooling_on();
        g_state = COOLING;
      } else {
        prints(PSTR("Cooling delay... %lu\r\n"), t_delay - tdiff);
      }

    } else {
      /* Temp is within range, do nothing */
    }
    break;

  default:
    /* Bad case... */
    break;
  } 
}
