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

struct rt {
  int32_t res;
  int32_t t;
};

struct rt rt_table[] = {
  { 437139, -55 },
  { 246574, -45 },
  { 143739, -35 },
  { 86390,  -25 },
  { 53417,  -15 },
  { 33916,  -5  },
  { 27280,  0,  },
  { 22073,  5   },
  { 14701,  15  },
  { 10000,  25  },
  { 6946,   35  },
  { 4914,   45  },
  { 3538,   55  },
  { 2589,   65  },
  { 1924,   75  },
  { 1451,   85  },
  { 1108,   95  },
  { 857,    105 },
  { 671,    115 },
  { 531,    125 }
};

static uint32_t adc2res(uint16_t adc)
{
  uint32_t res;
  uint32_t tmp = 1023UL*SCALE_FACTOR/adc - 1 * SCALE_FACTOR; 

  res = RES*SCALE_FACTOR/tmp;
  return res;
}

static int32_t adc2temp_v2(uint16_t adc)
{
  int32_t res = adc2res(adc);
  unsigned int i = 0; 
  int32_t k, m, t;

  while ((rt_table[i].res >= res) && (i < sizeof(rt_table)/sizeof(rt_table[0]))) {
    if (res == rt_table[i].res)
      return rt_table[i].t;
    ++i;
  }

  if (i == sizeof(rt_table)/sizeof(rt_table[0]))
    return 0;

  if (i) {
    k = (rt_table[i].t - rt_table[i-1].t) / (rt_table[i].res - rt_table[i-1].res);
    m = rt_table[i].t - k * rt_table[i].res;
    t = k*res + m;
    prints(PSTR("Temperature is: %ld\r\n"), t);
    return t;
  } else {
    prints(PSTR("Invalid resistance: %lu\r\n"), res);
  }

  return 0;
}

float temperature_get(void)
{
  uint16_t adc = adc_get();

  return adc2temp(adc);
}

