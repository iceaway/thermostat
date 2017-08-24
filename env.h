#ifndef __ENV_H
#define __ENV_H

void env_clear(void);
int env_set(char const *var, char const *val);
int env_get(char const *var, char *value, size_t size);
void env_dump(void);

#endif
