#include "ctrl.h"
#include "env.h"
#include "adc.h"
#include "temperature.h"
#include "heater.h"
#include "main.h"

enum heater_state {
  HEATER_OFF,
  HEATER_ON
};

static int g_enabled = 0;
static int g_state = HEATER_OFF;

void ctrl_enable(void)
{
  g_enabled = 1;
}

void ctrl_disable(void)
{
  g_enabled = 0;
}

int ctrl_status(void)
{
  return g_enabled;
}

void ctrl_update(void)
{
  char tmp[16];
  float t_set;
  float t_hyst;
  float temp = temperature_get();

  if (!g_enabled) {
    return;
  }

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

  /* temp = temp_get(); */
  if ((temp <= (t_set - t_hyst)) && (g_state == HEATER_OFF)) {
    /* turn on heater */
    prints("Turning ON heater\r\n");
    g_state = HEATER_ON;
    heater_on();
  } else if ((temp >= (t_set + t_hyst)) && (g_state == HEATER_ON)) {
    /* turn off heater */
    prints("Turning OFF heater\r\n");
    g_state = HEATER_OFF;
    heater_off();
  } else {
    /* between temperature limits, do nothing */
  }
}
