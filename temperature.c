#include <math.h>
#include <avr/pgmspace.h>
#include "temperature.h"
#include "adc.h"
#include "env.h"
#include "prints.h"
#include "pin.h"

/* NTC values */
#define RES     10000UL /* Series resistor in Ohm */

#define SCALE_FACTOR  10000UL

struct rt {
  int32_t res;
  int32_t t;
};

struct rt rt_table[] = {
  { 437139, -55 * SCALE_FACTOR },
  { 246574, -45 * SCALE_FACTOR },
  { 143739, -35 * SCALE_FACTOR },
  { 86390,  -25 * SCALE_FACTOR },
  { 53417,  -15 * SCALE_FACTOR },
  { 33916,  -5  * SCALE_FACTOR },
  { 27280,  0 * SCALE_FACTOR   },
  { 22073,  5 * SCALE_FACTOR   },
  { 14701,  15 * SCALE_FACTOR  },
  { 10000,  25 * SCALE_FACTOR  },
  { 6946,   35 * SCALE_FACTOR  },
  { 4914,   45 * SCALE_FACTOR  },
  { 3538,   55 * SCALE_FACTOR  },
  { 2589,   65 * SCALE_FACTOR  },
  { 1924,   75 * SCALE_FACTOR  },
  { 1451,   85 * SCALE_FACTOR  },
  { 1108,   95 * SCALE_FACTOR  },
  { 857,    105 * SCALE_FACTOR },
  { 671,    115 * SCALE_FACTOR },
  { 531,    125 * SCALE_FACTOR }
};


void temperature_init_gpio(void)
{
  /* Set ADC input as tri-stated (input, no pull-up) */
  pin_input(0, 0);
}

static uint32_t adc2res(uint16_t adc)
{
  uint32_t res;
  uint32_t tmp = 1023UL*SCALE_FACTOR/adc - 1 * SCALE_FACTOR; 

  res = RES*SCALE_FACTOR/tmp;
  return res;
}

static int32_t adc2temp(uint16_t adc)
{
  int32_t res = adc2res(adc);
  unsigned int i = 0; 
  int32_t k, m, t;

  while ((rt_table[i].res >= res) && (i < sizeof(rt_table)/sizeof(rt_table[0]))) {
    if (res == rt_table[i].res)
      return rt_table[i].t;
    ++i;
  }

  if ((i == 0) || (i == sizeof(rt_table)/sizeof(rt_table[0]))) {
    prints(PSTR("Invalid resistance: %lu\r\n"), res);
    return 0;
  }

  k = (rt_table[i].t - rt_table[i-1].t) / (rt_table[i].res - rt_table[i-1].res);
  m = rt_table[i].t - k * rt_table[i].res;
  t = k*res + m;
  /* Temp is scaled up by SCALE_FACTOR */
  return t;
}

int32_t temperature_get(void)
{
  uint16_t adc = adc_get();
  int32_t temp = adc2temp(adc);
  /* Return temperature scaled up by 10 to give one decimal accuracy */
  temp = (temp + 5000) / (SCALE_FACTOR/10);
  return temp;
}

