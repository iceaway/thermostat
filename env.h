#ifndef __ENV_H
#define __ENV_H
#include <stdlib.h>

#define MAX_NAME_LEN    31
#define MAX_VALUE_LEN   31

void env_clear(void);
int env_set(char const *var, char *val);
int env_get(char const *var, char *value, size_t size);
void env_dump(void);
void env_save(void);
void env_restore(void);

#endif
