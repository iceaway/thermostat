#include <math.h>
#include "temperature.h"
#include "adc.h"
#include "env.h"
#include "pin.h"

/* NTC values */
#define RES     10000UL /* Series resistor in Ohm */
#define BETA    3435.0f /* Beta coefficient for steinhart equation */
#define NOMTEMP 25.0f   /* Nominal temperature of temp sensor */

#define BETA_MIN  3000.0f /* Sanity check, min allowed value for BETA */
#define BETA_MAX  4000.0f /* Sanity check, max allowed value for BETA */

#define SCALE_FACTOR  10000UL

void temperature_init_gpio(void)
{
  /* Set ADC input as tri-stated (input, no pull-up) */
  pin_input(0, 0);
}

static float adc2temp(uint16_t val)
{
  float tmp2;
  float temp;
  float beta = BETA;
  char tmp[16];


  if (env_get("BETA", tmp, sizeof(tmp)) > 0) {
    beta = atof(tmp);
    if ((beta < BETA_MAX) && (beta > BETA_MIN))
      beta = BETA;
  }

  /* This code is pretty much taken directly from adafruits article on
   * using a thermistor.
   */
  tmp2 = 1023.0f / (float)val - 1.0f;
  tmp2 = RES / tmp2;

  temp = tmp2 / (float)RES;
  temp = log(temp);
  temp /= BETA;
  temp += 1.0f / (NOMTEMP + 273.15f);
  temp = 1.0f / temp;
  temp -= 273.15f;

  return temp;
}

static uint32_t adc2res(uint16_t adc)
{
  uint32_t res;
  uint32_t tmp = 1023UL*SCALE_FACTOR/adc - 1 * SCALE_FACTOR; 

  res = RES*SCALE_FACTOR/tmp;
  return res;
}

float temperature_get(void)
{
  uint16_t adc = adc_get();

  return adc2temp(adc);
}

