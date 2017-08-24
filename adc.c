#include <avr/io.h>
#include "adc.h"


void adc_init(void)
{
  /* PF0 = Analog pin = A0 */

  /* Enable ADC */
  ADCSRA = (1 << ADEN);

  /* Set prescaler to clk/128 to get ADC Clock of 125 kHz */
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}


uint16_t adc_get(void)
{
  /* TODO: ADC channel selection */

  /* Start conversion */
  ADCSRA |= (1 << ADSC);

  /* Wait for conversion to complete */
  while (!(ADCSRA & (1 << ADIF))) { }

  return ADC;
}
