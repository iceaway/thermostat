#include <string.h>
#include "main.h"
#include "sched.h"

#define MAX_TASKS 16
#define MAX_NAME_LEN  8

struct task {
  char name[MAX_NAME_LEN];
  uint32_t interval;
  uint32_t lastrun;
  void (*sched_fn)(void);
  uint8_t enabled;
  uint8_t run;
};

static struct task g_tasks[MAX_TASKS] = { 0 };

int sched_add_task(void (*sched_fn)(void), uint8_t enabled, uint32_t interval, char *name)
{
  int i = 0;
  
  if (!sched_fn || (interval == 0))
    return -1;

  for (i = 0; i < MAX_TASKS; ++i) {
    if (g_tasks[i].sched_fn == NULL) {
      g_tasks[i].sched_fn = sched_fn;
      g_tasks[i].enabled = enabled;
      g_tasks[i].interval = interval;
      g_tasks[i].lastrun = get_ticks();
      memset(g_tasks[i].name, 0, MAX_NAME_LEN);
      strncpy(g_tasks[i].name, name, MAX_NAME_LEN-1);
      return i;
    }
  }

  return -1;
}

int sched_delete_task(uint8_t id)
{
  if (id < MAX_TASKS) {
    memset(&g_tasks[id], 0, sizeof(g_tasks[id]));
    return 0;
  } else {
    return -1;
  }
}

int sched_enable_task(uint8_t id)
{
  if ((id < MAX_TASKS) && g_tasks[id].sched_fn) {
    g_tasks[id].enabled = 1;
    return 0;
  } else {
    return -1;
  }
}

int sched_disable_task(uint8_t id)
{
  if ((id < MAX_TASKS) && g_tasks[id].sched_fn) {
    g_tasks[id].enabled = 0;
    return 0;
  } else {
    return -1;
  }
}

void sched_update(void)
{
  int i;
  for (i = 0; i < MAX_TASKS; ++i) {
    if (g_tasks[i].sched_fn && g_tasks[i].enabled) {
      if ((get_ticks() - g_tasks[i].lastrun) >= g_tasks[i].interval) {
        g_tasks[i].run = 1;
      }
    }
  }
}

void sched_runtasks(void)
{
  int i;
  for (i = 0; i < MAX_TASKS; ++i) {
    if (g_tasks[i].sched_fn && g_tasks[i].enabled && g_tasks[i].run) {
      g_tasks[i].sched_fn();
    }
  }
}
