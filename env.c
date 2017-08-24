#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "env.h"
#include "main.h"

#define ENV_SIZE        128
#define EEPROM_INIT     0xAA

#define MIN(a,b)  ((a) < (b) ? (a) : (b))

static char env[ENV_SIZE] = { 0 };

/****************************************************************************
* Name: env_get 
*
* Description:
*   Get a stored environment variable. The returned value is always zero
*   terminated.
*
* Input Parameters:
*   var       - Variable to get
*   value     - Pointer to buffer where the variable value should be stored
*   size      - Size of the value buffer
*
* Returned Value:
* <=0 - Failure
*  >0 - Length of the returned string 
*
* Assumptions/Limitations:
*
****************************************************************************/
int env_get(char const *var, char *value, size_t size)
{
  char *val = NULL;
  char *valp = NULL;
  char *startp = NULL;
  char *endp = NULL;
  int n;

  val = env;
  while ((val = strstr(val, var)) != NULL) {
    valp = val;
    ++val;
  }

  if (valp) {
    startp = strchr(valp, '=');
    ++startp;
    endp = strchr(valp, ';');
    if (!endp || !startp) {
      prints("Error in environment!\r\n");
      return 0;
    } else {
      n = MIN(endp - startp, (int)size - 1);
      strncpy(value,
          startp,
          n);

      value[n] = '\0';
      return strlen(value);
    }
  } else {
    return -1;
  }
}

/****************************************************************************
* Name: env_set 
*
* Description:
*   Set an environment variable.
*
* Input Parameters:
*   var       - Variable to set
*   value     - Variable value to set
*
* Returned Value:
*   0 - Failure
*  >0 - Length of the set string 
*
* Assumptions/Limitations:
*
****************************************************************************/
int env_set(char const *var, char const *val)
{
  unsigned int varlen = strlen(var);
  unsigned int vallen = strlen(val);
  /* TODO: Make it handle updates of existing variables */

  if ((vallen + varlen) > (ENV_SIZE - strlen(env) - 2)) {
    prints("Environment is full\r\n");
    return 0;
  } else if (varlen > 0) {
    return snprintf(env+strlen(env), ENV_SIZE - strlen(env), "%s=%s;", var, val);
  } else {
    prints("No variable given\r\n");
    return 0;
  }
}
/****************************************************************************
* Name: env_clear
*
* Description:
*   Clear the environment. Remove all set variables from env.
*
* Input Parameters:
*   None
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*
****************************************************************************/
void env_clear(void)
{
  memset(env, 0, sizeof(env));
}

/****************************************************************************
* Name: env_dump
*
* Description:
*   Dump the environment to the serial port
*
* Input Parameters:
*   None
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*
****************************************************************************/
void env_dump(void)
{
  prints(env);
}

/****************************************************************************
* Name: env_save
*
* Description:
*   Save the environment to EEPROM
*
* Input Parameters:
*   None
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*
****************************************************************************/
void env_save(void)
{
  /* TODO: Check if anything has changed before writing */
  eeprom_write_block(env, (void *)1, ENV_SIZE);
  prints("Environment saved\r\n");
}

/****************************************************************************
* Name: env_restore
*
* Description:
*   Restore the environment from EEPROM
*
* Input Parameters:
*   None
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*
****************************************************************************/
void env_restore(void)
{
  if (eeprom_read_byte(0) == EEPROM_INIT) {
    eeprom_read_block(env, (void *)1, ENV_SIZE);
    prints("Environment restored\r\n");
  } else {
    prints("No environment exists in EEPROM\r\n");
  }
}


