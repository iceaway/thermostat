#include "ctrl.h"

static int g_enabled = 0;

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

int ctrl_update(void)
{
}
