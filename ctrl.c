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
static unsigned int g_laston = 0;

void ctrl_enable(void)
{
  g_state = IDLE;
  g_laston = get_ticks();
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
  int t_delay;
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

  if (t_delay < 0)
      return;

  switch (g_state) {
  case OFF:
    break;

  case COOLING:
    g_laston = get_ticks();
    if (temp < t_set) {
      cooling_off();
      g_state = IDLE;
    }
    break;

  case HEATING:
    if (temp > t_set) {
      heater_off();
      g_state = IDLE;
    }
    break;

  case IDLE:
    if (temp <= (t_set - t_hyst)) {
      heater_on();
      g_state = HEATING;
    } else if (temp >= (t_set + t_hyst)) {
      if ((get_ticks() - g_laston ) >= (t_delay*1000)) {
        cooling_on();
        g_state = COOLING;
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
