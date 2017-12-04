#ifndef __CTRL_H
#define __CTRL_H

enum ctrl_state {
  OFF,
  IDLE,
  HEATING,
  COOLING
};

void ctrl_enable(void);
void ctrl_disable(void);
int ctrl_status(void);
void ctrl_update(void);
int ctrl_state(void);

#endif
