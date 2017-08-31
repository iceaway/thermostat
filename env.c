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
  int tmp;
  /* TODO: Make it handle updates of existing variables */

  if ((vallen + varlen) > (ENV_SIZE - strlen(env) - 2)) {
    prints("Environment is full\r\n");
    return 0;
  } else if (varlen > 0) {
    tmp = snprintf(env+strlen(env), ENV_SIZE - strlen(env), "%s=%s;", var, val);
    env_save();
    return tmp;
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
  env_save();
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
  /* TODO: Check if anything has changed before writing.
   * EEPROM v2:
   * Byte 0 - INIT
   * Byte 1-2 - Env size
   * Byte 3-n - Env
   */
  uint16_t envlen = strlen(env);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)1, (envlen & 0xff00) >> 8);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)2, envlen & 0x00ff);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_block(env, (void *)3, envlen);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)0, EEPROM_INIT);
  prints("Environment saved, wrote %ud bytes\r\n", envlen);
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
  uint16_t envlen;
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  if (eeprom_read_byte((void *)0) == EEPROM_INIT) {
    eeprom_busy_wait(); /* Wait until eeprom is ready */
    envlen = eeprom_read_byte((void *)1) << 8;
    eeprom_busy_wait(); /* Wait until eeprom is ready */
    envlen |= eeprom_read_byte((void *)2);
    eeprom_read_block(env, (void *)3, envlen);
    env[envlen] = '\0';
    prints("Environment restored, read %ld bytes\r\n", envlen);
  } else {
    prints("No environment exists in EEPROM\r\n");
  }
}


