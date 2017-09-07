#include <math.h>
#include "temperature.h"
#include "adc.h"
#include "env.h"

/* NTC values */
#define RES     10000UL /* Series resistor in Ohm */
#define BETA    3435.0f /* Beta coefficient for steinhart equation */
#define NOMTEMP 25.0f   /* Nominal temperature of temp sensor */

#define BETA_MIN  3000.0f /* Sanity check, min allowed value for BETA */
#define BETA_MAX  4000.0f /* Sanity check, max allowed value for BETA */

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

float temperature_get(void)
{
  uint16_t adc = adc_get();

  return adc2temp(adc);
}

