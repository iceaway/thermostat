#include <string.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "env.h"
#include "main.h"
#include "prints.h"

#define ENV_SIZE        64
#define EEPROM_INIT     0xAB
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)
#define EEPROM_SIZE     1024
#elif defined (__AVR_ATmega2560__)
#define EEPROM_SIZE     4096
#else
#error Unsupported MCU
#endif

#define MIN(a,b)  ((a) < (b) ? (a) : (b))

static char g_env[ENV_SIZE] = { 0 };
static char g_env_gc[ENV_SIZE] = { 0 };

static int env_get_internal(char *env, char const *var, char *value, size_t size);

/****************************************************************************
* Name: garbage_collect
*
* Description:
*   Remove all old entries of variables in the environment.
*
* Input Parameters:
*   None
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*   None
*
****************************************************************************/
static void garbage_collect(void)
{
  char *p;
  char *endp;
  char varname[MAX_NAME_LEN+1];
  char value[MAX_VALUE_LEN+1];
  int end;
  int len;
  int gcidx = 0;
  unsigned int pre_size;
  unsigned int post_size;
  /* General outline of the process:
   * 1. Find the first variable in the environment. Save the varname and value
   *    to a temporary environment copy. 
   * 2. Proceed to the next variable. Check if it exists 
   *    in the temporary environment. If not, copy it. Otherwise ignore it.
   * 3. When the end of the original environment is reached, we are
   *    finished.
   */

  end = strlen(g_env);
  pre_size = end;
  if (end > ENV_SIZE) {
    prints(PSTR("Env is somehow too big.\r\n"));
    return;
  }

  if (end == 0) {
    prints(PSTR("Env is empty, no need to gc\r\n"));
    return;
  }

  memset(g_env_gc, 0, sizeof(g_env_gc));

  p = g_env; 
  prints(PSTR("Current env: '%s'\r\n"), g_env);
  /* TODO: This does not seem to work anymore, gets stuck in endless loop */
  while (p) {
    endp = strchr(p, '=');
    /* endp now points to the END of the first var=value pair */
    if (endp) {
      len = endp - p;
      if (len && (len < MAX_NAME_LEN)) {
        memset(varname, 0, sizeof(varname));
        memcpy(varname, p, len);
        if (env_get_internal(g_env_gc, varname, NULL, 0) < 0) {
          env_get(varname, value, MAX_VALUE_LEN);
          gcidx += snprintf(g_env_gc + gcidx,
                            sizeof(g_env_gc) - gcidx,
                            "%s=%s;",
                            varname,
                            value);

        } else {
          /* var is already in temporary environment, ignore it */
        }
      } else {
        /* either var name is zero or name is too long, neither should 
         * be possible. Clear env?
         */
      }
    } else {
      /* End of environment reached! */
      prints(PSTR("Found the end of the environment, replacing old environment\r\n"));
      memcpy(g_env, g_env_gc, gcidx);
      g_env[gcidx] = '\0';
      env_save();
      post_size = gcidx;
      prints(PSTR("Garbage collect saved %d bytes\r\n"), pre_size - post_size);
      break;
    }
    p = strchr(p+1, ';');
    if (p)
      ++p;
  }
}

int env_get(char const *var, char *value, size_t size)
{
  return env_get_internal(g_env, var, value, size);
}

/****************************************************************************
* Name: env_get_internal
*
* Description:
*   Get a stored environment variable. The returned value is always zero
*   terminated.
*
* Input Parameters:
*   env       - pointer to environment to get variable from
*   var       - Variable to get
*   value     - Pointer to buffer where the variable value should be stored
*   size      - Size of the value buffer
*
* Returned Value:
*  <0 - Failure
*  >=0 - Length of the returned string 
*
* Assumptions/Limitations:
*
****************************************************************************/
static int env_get_internal(char *env, char const *var, char *value, size_t size)
{
  char *val = NULL;
  char *valp = NULL;
  char *startp = NULL;
  char *endp = NULL;
  int n = 0;

  /* TODO: We need to check here that the substring returned is actually
   * a variable and not the contents of a different variable.
   */
  val = env;
  while ((val = strstr(val, var)) != NULL) {
    if ((val == env) || (*(val-1) == ';'))
      valp = val;
    ++val;
  }

  if (valp) {
    startp = strchr(valp, '=');
    ++startp;
    endp = strchr(valp, ';');
    if (!endp || !startp) {
      prints(PSTR("Error in environment!\r\n"));
      return 0;
    } else {
      if (value && size) {
        n = MIN(endp - startp, (int)size - 1);
        strncpy(value, startp, n);
        value[n] = '\0';
      }
      return n;
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
int env_set(char const *var, char *val)
{
  int varlen = (int)strlen(var);
  int vallen = (int)strlen(val);
  char *p;
  int tmp;
  
  if ((vallen + varlen) > (ENV_SIZE - (int)strlen(g_env) - 2)) {
    prints(PSTR("Environment is full, garbage collecting.\r\n"));
    /* Do garbage collection here and remove all old versions of 
     * variables. Try to save new value again.
     */
    garbage_collect();
    /* TODO: If var still does not fit - this loops endlessly */
    return env_set(var, val);
  } else if (varlen > 0) {
    /* Convert ; (reserved as delimiter) to . */
    if (strchr(var, ';') || strchr(var, '=')) {
      prints(PSTR("Not allowed to have ';' or '=' in the variable name.\r\n"));
      return -1;
    }

    p = val;
    /* replace illegal characters in values */
    while ((p = strchr(p, ';')))
      *p = '.';

    p = val;
    while ((p = strchr(p, '=')))
      *p = '.';

    tmp = snprintf(g_env+strlen(g_env), ENV_SIZE - strlen(g_env), "%s=%s;", var, val);
    prints(PSTR("wrote %d bytes to env\r\n"), tmp);
    env_save();
    return tmp;
  } else {
    prints(PSTR("No variable given\r\n"));
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
  memset(g_env, 0, sizeof(g_env));
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
  prints(g_env);
  prints("\r\n");

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
  uint16_t envlen = strlen(g_env);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)1, (envlen & 0xff00) >> 8);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)2, envlen & 0x00ff);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_block(g_env, (void *)3, envlen);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  eeprom_write_byte((void *)0, EEPROM_INIT);
  eeprom_busy_wait(); /* Wait until eeprom is ready */
  prints(PSTR("Environment saved, wrote %hu bytes\r\n"), envlen);
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
    if (envlen <= EEPROM_SIZE) {
      prints(PSTR("Reading %hu bytes from EEPROM\r\n"), envlen);
      eeprom_read_block(g_env, (void *)3, envlen);
      g_env[envlen] = '\0';
      prints(PSTR("Environment restored, read %hu bytes\r\n"), envlen);
    }  else {
      prints(PSTR("Invalid size of environment: %lu\r\n"), envlen);
    }
  } else {
    prints(PSTR("No environment exists in EEPROM\r\n"));
  }
}

