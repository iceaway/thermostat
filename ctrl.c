#include "ctrl.h"

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
  float t_hi;
  float t_lo;
  float temp;

  if (env_get("T_HI", tmp, sizeof(tmp)) <= 0)
    return;
  
  t_hi = atof(tmp);
  if ((t_hi < 0.0f) || (t_hi > 100.0f))
    return;

  if (env_get("T_LO", tmp, sizeof(tmp)) <= 0)
    return;
  
  t_lo = atof(tmp);
  if ((t_lo < 0.0f) || (t_lo > 100.0f) || (t_lo > t_hi))
    return;


  /* temp = temp_get(); */
  if ((temp <= t_lo) && (g_state == HEATER_OFF)) {
    /* turn on heater */
    prints("Turning ON heater\r\n");
    g_state = HEATER_ON;
  } else if ((temp >= t_hi) && (g_state == HEATER_ON)) {
    /* turn off heater */
    prints("Turning OFF heater\r\n");
    g_state = HEATER_OFF;
  } else {
    /* between temperature limits, do nothing */
  }
}