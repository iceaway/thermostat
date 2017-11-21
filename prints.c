#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "prints.h"

#define PRINTS_BUFSIZE  128

/****************************************************************************
* Name: prints
*
* Description:
*   Sends a formatted string to the debug terminal
*
* Input Parameters:
*   fmt       - Format string (printf style)
*   ...       - Values used in the format string
*
* Returned Value:
*   Number of bytes sent
*
* Assumptions/Limitations:
*
****************************************************************************/
int prints(const __memx char *fmt, ...)
{
  va_list ap;
  char buf[PRINTS_BUFSIZE] = { 0 };
  int size;

  va_start(ap, fmt);
  if (__builtin_avr_flash_segment(fmt) < 0) {
    size = vsnprintf(buf, sizeof(buf), (char *)fmt, ap);
  } else {
    size = vsnprintf_P(buf, sizeof(buf), fmt, ap);
  }
  va_end(ap);

  print_string(buf);
  return size;
}
