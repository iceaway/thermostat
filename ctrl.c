#include "ctrl.h"
#include "env.h"
#include "adc.h"
#include "temperature.h"
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
  float t_hi;
  float t_lo;
  float temp; 
  uint16_t adc = adc_get();
  temp = adc2temp(adc);

  if (!g_enabled) {
    g_state = HEATER_OFF;
    pin_low(37);
    return;
  }

  if (env_get("T_HI", tmp, sizeof(tmp)) <= 0) {
    g_state = HEATER_OFF;
    pin_low(37);
    return;
  }
  
  t_hi = atof(tmp);
  if ((t_hi < 0.0f) || (t_hi > 100.0f)) {
    g_state = HEATER_OFF;
    pin_low(37);
    return;
  }

  if (env_get("T_LO", tmp, sizeof(tmp)) <= 0) {
    g_state = HEATER_OFF;
    pin_low(37);
    return;
  }
  
  t_lo = atof(tmp);
  if ((t_lo < 0.0f) || (t_lo > 100.0f) || (t_lo > t_hi)) {
    g_state = HEATER_OFF;
    pin_low(37);
    return;
  }

  /* temp = temp_get(); */
  if ((temp <= t_lo) && (g_state == HEATER_OFF)) {
    /* turn on heater */
    prints("Turning ON heater\r\n");
    g_state = HEATER_ON;
    pin_high(37);
  } else if ((temp >= t_hi) && (g_state == HEATER_ON)) {
    /* turn off heater */
    prints("Turning OFF heater\r\n");
    g_state = HEATER_OFF;
    pin_low(37);
  } else {
    /* between temperature limits, do nothing */
  }
}
