#ifndef __SCHED_H
#define __SCHED_H

#include <stdint.h>

int sched_add_task(void (*sched_fn)(void), uint8_t enabled, uint32_t interval, char *name);
int sched_delete_task(uint8_t id);
int sched_enable_task(uint8_t id);
int sched_disable_task(uint8_t id);
void sched_update(void);
void sched_runtasks(void);

#endif
