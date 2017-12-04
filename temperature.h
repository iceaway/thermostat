#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#include <stdint.h>

int32_t temperature_get(void);
void temperature_init_gpio(void);

#endif
