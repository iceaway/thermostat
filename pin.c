#include <stdlib.h>
#include <avr/io.h>
#include "main.h"
#include "pin.h"

enum pinmode {
  PIN_HIGH,
  PIN_LOW,
  PIN_FLOAT
};

struct pinport {
  uint8_t const pin_no;             /* Arduino pin number */
  volatile uint8_t *port;     /* PORTx register */
  volatile uint8_t *ddr;      /* DDRx register */
  volatile uint8_t *pin;      /* PINx register */
  uint8_t const bit;                /* Pxn bit */
};

struct pinport pinportmap[] = {
  { 0,  &PORTF, &DDRF, &PINF, PF0 }, /* ANALOG PIN ZERO !!! */
  { 19, &PORTD, &DDRD, &PIND, PD2 },
  { 20, &PORTD, &DDRD, &PIND, PD1 },
  { 21, &PORTD, &DDRD, &PIND, PD0 },
  { 22, &PORTA, &DDRA, &PINA, PA0 },
  { 23, &PORTA, &DDRA, &PINA, PA1 },
  { 24, &PORTA, &DDRA, &PINA, PA2 },
  { 25, &PORTA, &DDRA, &PINA, PA3 },
  { 27, &PORTA, &DDRA, &PINA, PA5 },
  { 29, &PORTA, &DDRA, &PINA, PA7 },
  { 32, &PORTC, &DDRC, &PINC, PC5 },
  { 33, &PORTC, &DDRC, &PINC, PC4 },
  { 34, &PORTC, &DDRC, &PINC, PC3 },
  { 35, &PORTC, &DDRC, &PINC, PC2 },
  { 36, &PORTC, &DDRC, &PINC, PC1 },
  { 37, &PORTC, &DDRC, &PINC, PC0 },
  { 38, &PORTD, &DDRD, &PIND, PD7 },
  { 39, &PORTG, &DDRG, &PING, PG2 },
  { 0,  NULL,   NULL,  NULL,  0 }
};

/****************************************************************************
* Name: get_pinport 
*
* Description:
*   Searches the pinportmap struct for a given pin number and returns a pointer
*   to the correct entry if it was found.
*
* Input Parameters:
*   pin        - Pin number to locate
*
* Returned Value:
*   Pointer to the found item in the pinportmap struct, or NULL if it was
*   unable to find the given pin.
* 
* Assumptions/Limitations:
*
****************************************************************************/
static struct pinport *get_pinport(int pin)
{
  int i = 0;

  /* Find the correct mapping between arduino pin and PORTx/Pxn definitions */
  while (pinportmap[i].port && (pinportmap[i].pin_no != pin)) { ++i; }
  if (pinportmap[i].port) {
    return &pinportmap[i];
  } else {
    return NULL;
  }
}

/****************************************************************************
* Name: pin_read 
*
* Description:
*   Read the digital value of a single pin
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*
* Returned Value:
*   0  - Low
*   1  - High
*  -1  - Pin not found
* 
* Assumptions/Limitations:
*
****************************************************************************/
int pin_read(int pin)
{
  int ret;
  int i = 0;

  /* Find the correct mapping between arduino pin and PORTx/Pxn definitions */
  while (pinportmap[i].port && (pinportmap[i].pin_no != pin)) { ++i; }

  if (pinportmap[i].port) {
    ret = (*pinportmap[i].pin & (1 << pinportmap[i].bit)) ? 1 : 0;
  } else {
    prints("Undefined pin no: %d\r\n", pin);
    ret = -1;
  }

  return ret;
}

/****************************************************************************
* Name: pin_inout
*
* Description:
*   Sets a pin as an input or output.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*   input     - 1 = Input, 0 = Output
*   pullup    - 1 = Activate pull-up (input only), 0 = Disable pull-up
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*
****************************************************************************/
static int pin_inout(int pin, int input, int pullup)
{
  int ret = 0;
  int i = 0;

  /* Find the correct mapping between arduino pin and PORTx/Pxn definitions */
  while (pinportmap[i].port && (pinportmap[i].pin_no != pin)) { ++i; }

  if (pinportmap[i].port) {
    if (input) {
      *pinportmap[i].ddr  &= ~(1 << pinportmap[i].bit);

      if (pullup) {
        *pinportmap[i].port |= (1 << pinportmap[i].bit);
      } else {
        *pinportmap[i].port &= ~(1 << pinportmap[i].bit);
      }
    } else {
      *pinportmap[i].ddr  |=  (1 << pinportmap[i].bit);
    }

    ret = pin;
  } else {
    prints("Undefined pin no: %d\r\n", pin);
    ret = 0;
  }

  return ret;
}

/****************************************************************************
* Name: pin_input
*
* Description:
*   Sets a pin as an input.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*   pullup    - 1 = Activate pull-up, 0 = Disable pull-up
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*
****************************************************************************/
int pin_input(int pin, int pullup)
{
  return pin_inout(pin, 1, pullup);
}

/****************************************************************************
* Name: pin_output
*
* Description:
*   Sets a pin as an output.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*
****************************************************************************/
int pin_output(int pin)
{
  return pin_inout(pin, 0, 0);
}

/****************************************************************************
* Name: pin_highlowfloat
*
* Description:
*   Sets a pin as high/low/floating.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*   mode      - Mode. PIN_LOW = low, PIN_HIGH = high, PIN_FLOAT = floating
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*   Setting a pin as floating also changes it to an input pin if it was
*   previously an output.
*
****************************************************************************/
static int pin_highlowfloat(int pin, int mode)
{
  int ret = 0;
  int i = 0;

  /* Find the correct mapping between arduino pin and PORTx/Pxn definitions */
  while (pinportmap[i].port && (pinportmap[i].pin_no != pin)) { ++i; }

  if (pinportmap[i].port) {
    /* Found */
    prints("Setting pin %d to mode %d\r\n", pin, mode); 
    switch (mode) {
    case PIN_LOW:
      *pinportmap[i].ddr  |=  (1 << pinportmap[i].bit);
      *pinportmap[i].port &= ~(1 << pinportmap[i].bit);
      ret = pin;
      break;

    case PIN_HIGH:
      *pinportmap[i].ddr  |= (1 << pinportmap[i].bit);
      *pinportmap[i].port |= (1 << pinportmap[i].bit);
      ret = pin;
      break;

    case PIN_FLOAT:
      *pinportmap[i].ddr  &= ~(1 << pinportmap[i].bit);
      *pinportmap[i].port &= ~(1 << pinportmap[i].bit);
      ret = pin;
      break;

    default:
      prints("Error: Undefined mode: %d\r\n", mode);
      ret = 0;
    }

  } else {
    prints("Undefined pin no: %d\r\n", pin);
    ret = 0;
  }

  return ret;
}

/****************************************************************************
* Name: pin_low
*
* Description:
*   Sets a pin low.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*   Setting a pin as floating also changes it to an input pin if it was
*   previously an output.
*
****************************************************************************/
int pin_low(int pin)
{
  return pin_highlowfloat(pin, PIN_LOW);
}

/****************************************************************************
* Name: pin_high
*
* Description:
*   Sets a pin high.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*
****************************************************************************/
int pin_high(int pin)
{
  return pin_highlowfloat(pin, PIN_HIGH);
}

/****************************************************************************
* Name: pin_float
*
* Description:
*   Sets a pin floating.
*
* Input Parameters:
*   pin       - Pin number (Arduino number)
*
* Returned Value:
*   0   -  Pin not found
*  >0   -  OK, pin number
* 
* Assumptions/Limitations:
*   Setting a pin as floating also changes it to an input pin if it was
*   previously an output.
*
****************************************************************************/
int pin_float(int pin)
{
  return pin_highlowfloat(pin, PIN_FLOAT);
}

