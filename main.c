#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "rbuf.h"
#include "ctrl.h"
#include "env.h"
#include "adc.h"
#include "temperature.h"
#include "sched.h"
#include "pin.h"
#include "heater.h"

#define PRINTS_BUFSIZE  128
#define MAX_ARGC        8 

#define MIN(a,b)  ((a) < (b) ? (a) : (b))

#define IS_INPUT(ddr, bit)  ((ddr) & (1 << (bit)) ? 0 : 1)
#define IS_OUTPUT(ddr, bit)  ((ddr) & (1 << (bit)) ? 1 : 0)

#define ASCII_DEL  0x7F
#define ASCII_BS   0x08

struct cmd {
  char const * const cmd;
  char const * const help;
  int (*callback_fn)(int argc, char *argv[]);
};

int cmd_test(int argc, char *argv[]);
int cmd_help(int argc, char *argv[]);
int cmd_set(int argc, char *argv[]);
int cmd_echo(int argc, char *argv[]);
int cmd_env(int argc, char *argv[]);
//int cmd_pin(int argc, char *argv[]);
int cmd_reboot(int argc, char *argv[]);
int cmd_time(int argc, char *argv[]);
int cmd_adc(int argc, char *argv[]);
int cmd_temp(int argc, char *argv[]);
int cmd_ctrl(int argc, char *argv[]);
int cmd_ramdump(int argc, char *argv[]);
int cmd_heat(int argc, char *argv[]);

static void echo(char data);
static int reboot(void);

static int g_echo = 1;
static struct rbuf g_rxbuf_terminal;
static struct rbuf g_txbuf_terminal;
static uint32_t g_ticks = 0;

const struct cmd commands[] = {
  { "help", "print help", cmd_help },
  { "test", "Test command", cmd_test },
  { "set",  "Set environment variables", cmd_set },
  { "echo", "Print environment variables", cmd_echo },
  { "env",  "Environment commands", cmd_env },
//  { "pin",  "Set pin no high/low", cmd_pin },
  { "time", "Display uptime in [ms]", cmd_time },
  { "ramdump", "Dump ram", cmd_ramdump },
  { "adc",   "Display raw ADC value", cmd_adc },
  { "ctrl",   "Enable/disable control loop", cmd_ctrl },
  { "heat", "Turn on/off heater", cmd_heat },
  { "temp", "Show current temperature", cmd_temp },
  { "reboot", "Reboot!", cmd_reboot },
  { NULL, NULL, NULL }
};

void color_stack(void) __attribute__ ((naked)) \
     __attribute__ ((section (".init3")));

extern uint8_t _end;
extern uint8_t __stack;

void color_stack(void)
{
  uint8_t *p = &_end;
  while (p <= &__stack)
    *p++ = 0xcc;
}

/****************************************************************************
 * Name: transmit 
 *
 * Description:
 *   Start transmission of USART data.
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
static void transmit(void)
{
  /* Enable the TX interrupt */
  UCSR0B |= (1 << UDRIE0);
  /* Trigger interrupt for first transmission */
  UCSR0A |= (1 << UDRE0);
}

/****************************************************************************
 * Name: print_char 
 *
 * Description:
 *   Transmit a single character on the debug terminal. Used for echo.
 *
 * Input Parameters:
 *   data        - Character to be transmitted
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *
 ****************************************************************************/
static void print_char(char data)
{
  rbuf_push(&g_txbuf_terminal, data);
  transmit();
}

uint32_t get_ticks(void)
{
  return g_ticks;
}

/* TIMER0 ISR: Tick counter. Executed every 1 ms */
ISR(TIMER0_COMPA_vect)
{
  ++g_ticks;
  if ((g_ticks % 400) == 0) {
    PORTC ^= (1 << PC7);
    PORTB ^= _BV(PORTB7);
  }
  sched_update();
}

/* USART0_UDRE ISR: 
 * Called when USART0 is ready to receive another byte in the transmit
 * register. 
 */
ISR(USART0_UDRE_vect)
{
  char tmp;
  if (rbuf_pop(&g_txbuf_terminal, &tmp)) {
    UDR0 = tmp;
  } else {
    /* Nothing more to send, disable interrupt */
    UCSR0B &= ~(1 << UDRIE0);
  }
}

/* USART0_RX ISR: Called when a byte is received on USART0 */
ISR(USART0_RX_vect)
{
  uint8_t tmp;

  tmp = UDR0;
  rbuf_push(&g_rxbuf_terminal, tmp);
}

/****************************************************************************
* Name: print_string 
*
* Description:
*   Sends a string on the given USART device
*
* Input Parameters:
*   dev       - USART device. 0 = Debug terminal, 1 = Test script
*   string    - The string to send
*
* Returned Value:
*   None
*
* Assumptions/Limitations:
*
****************************************************************************/
static void print_string(char *string)
{
  struct rbuf *rb = NULL;

  rb = &g_txbuf_terminal;

  while (*string) {
    while (rbuf_push(rb, *string) != 1);
    string++;
  } 

  transmit();
}

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
int prints(const char *fmt, ...)
{
  va_list ap;
  char buf[PRINTS_BUFSIZE];
  int size;

  va_start(ap, fmt);
  size = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  print_string(buf);
  return size;
}

int cmd_heat(int argc, char *argv[])
{
  if (argc < 2) {
    prints("Not enough arguments. Usage: %s [ on | off ]\r\n",
           argv[0]);
    return -1;
  }

  if (strcmp(argv[1], "on") == 0) {
    prints("Enabling heater\r\n");
    heater_on();
  } else if (strcmp(argv[1], "off") == 0) {
    prints("Disabling heater\r\n");
    heater_off();
  } else {
    prints("Invalid argument: %s\r\n", argv[1]);
    return -1;
  }

  return 0;
}

int cmd_ctrl(int argc, char *argv[])
{
  if (argc < 2) {
    prints("Not enough arguments. Usage: %s [ enable | disable | status ]\r\n",
           argv[0]);
    return -1;
  }

  if (strcmp(argv[1], "enable") == 0) {
    prints("Enabling control loop\r\n");
    ctrl_enable();
  } else if (strcmp(argv[1], "disable") == 0) {
    prints("Disabling control loop\r\n");
    ctrl_disable();
  } else if (strcmp(argv[1], "status") == 0) {
    prints("Control loop is: %s\r\n",
           ctrl_status() == 0 ? "Disabled" : "Enabled");
  } else {
    prints("Invalid argument: %s\r\n", argv[1]);
    return -1;
  }

  return 0;
}

void print_temp(void)
{
  float temp = temperature_get();
  prints("Temperature: %d\r\n", (int)round(temp*10));
}

int cmd_temp(int argc, char *argv[])
{
  static uint8_t id;
  float temp;
  int interval;
  
  if (argc > 1) {
    interval = atoi(argv[1]);
    if (interval > 0) {
      prints("Displaying temperature with %d ms interval.");
      id = sched_add_task(print_temp, 1, interval, "temp");
    } else {
      sched_delete_task(id);
    }
  }

  temp = temperature_get();
  prints("Temperature: %d\r\n", (int)round(temp*10));

  return 0;
}

int cmd_adc(int argc, char *argv[])
{
  uint16_t val;
  uint32_t voltage;
  uint32_t tmp;
  uint32_t resistance;

  val = adc_get();
  prints("ADC Value: %u\r\n", val);
  
  voltage = 3300UL * (uint32_t)val / 1023UL; /* Voltage in mV */
  prints("Voltage: %lu mV\r\n", voltage);

  tmp = (1023UL * 1000UL / val) - 1000UL;
  resistance = 1000UL * 10000UL  / tmp;

  prints("Resistance: %lu Ohm\r\n", resistance);

  return 0;
}

int cmd_ramdump(int argc, char *argv[])
{
  uint16_t start;
  uint16_t n;
  uint16_t row;
  uint8_t *endptr;
  uint8_t *ptr;

  prints("end = %04x, stack = %04x\r\n", &_end, &__stack);

  if (argc >= 3) {
    start = strtoul(argv[1], NULL, 16);
    n = strtoul(argv[2], NULL, 16);

    if ((start < 0x200) || (start > 0x21FF)) {
      prints("Invalid start address\r\n");
      return 0;
    }

    if ((start + n) > 0x21FF) {
      prints("Range is outside RAM space\r\n");
      return 0;
    }

    endptr = (uint8_t *)(start + n);
    row = 0;
    ptr = (uint8_t *)start;
    while (ptr <= endptr) {
      if ((row % 8) == 0)
        prints("\r\n%04x    ", ptr);
      prints("%02x ", *ptr); 
      ++row;
      ++ptr;
    }
    prints("\r\n");
  } else {
    prints("Not enough arguments. Usage: %s <start_addr> <size>\r\n", argv[0]);
  }

  return 0;
}

int cmd_time(int argc, char *argv[])
{
  prints("Current uptime is: %lu ms\r\n", g_ticks);
  return 0;
}

#if 0
int cmd_pin(int argc, char *argv[])
{
  int pin;
  struct pinport *pp;
  
  if ((argc >= 2) && (strcmp(argv[1], "list") == 0)) {
    pp = &pinportmap[0];
    prints("Available pins:\r\n");
    while (pp->port) {
      prints("%u\r\n", pp->pin_no);
      ++pp;
    }
  } else if (argc >= 3) {
    pin = atoi(argv[1]);
    
    if ((pin < 0) | (pin > 53)) {
      prints("Invalid pin number: %d\r\n", pin);
      return 0;
    }

    if ((strcmp(argv[2], "high") == 0) || 
        (strcmp(argv[2], "h") == 0)) {
      if (pin_high(pin) == 0) {
        prints("Unavailable pin: %d\r\n", pin);
      }
    } else if ((strcmp(argv[2], "low") == 0) || 
               (strcmp(argv[2], "l") == 0)) {
      if (pin_low(pin) == 0) {
        prints("Unavailable pin: %d\r\n", pin);
      }
    } else {
      prints("Invalid mode: '%s'\r\n", argv[2]);
      return 0;
    }
  } else {
    prints("Not enough arguments\r\n");
  }

  return 0;
}
#endif

int cmd_set(int argc, char *argv[])
{
  char *eq;
  char *var;
  char *val;

  if (argc < 2) {
    prints("Not enough arguments\r\n");
  } else {
    if ((eq = strstr(argv[1], "=")) != NULL) {
      *eq = '\0';
      var = argv[1];
      val = eq+1;
      env_set(var, val);
      *eq = '=';
    }
  }

  return 0;

}

int cmd_env(int argc, char *argv[])
{
  if (argc >= 2) {
    if (strcmp(argv[1], "dump") == 0) {
      env_dump();
      prints("\r\n");
    } else if (strcmp(argv[1], "clear") == 0) {
      env_clear();
    } else {
      prints("Unknown argument.\r\n");
    }
  } else {
    prints("Not enough arguments\r\n");
  }
  return 0;
}

int cmd_reboot(int argc, char *argv[])
{
  reboot();
  return 0;
}

int cmd_echo(int argc, char *argv[])
{
  char *var = NULL;
  char value[32] = { 0 };
  int len;

  if (argc < 2) {
    prints("Not enough arguments\r\n");
  } else {
    if (argv[1][0] == '$') {
      var = argv[1]+1;
      if ((len = env_get(var, value, sizeof(value))) >= 0)
        prints("%s (%d)\r\n", value, len);
      else
        prints("Variable does not exist\r\n");

    } else {
      prints(argv[1]);
    }
  }

  return 0;
}

int cmd_test(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  prints("cunter value: %u\r\n", TCNT1);
  prints("overflow reg: %02x\r\n", TIFR1);
  prints("timsk1 reg:   %02x\r\n", TIMSK1);
  return 0;
}

int cmd_help(int argc, char *argv[])
{
  const struct cmd *p;
  (void)argc;
  (void)argv;
  p = &commands[0];
  prints("Available commands:\r\n");
  while (p->cmd) {
    prints("%-15s - %s\r\n", p->cmd, p->help);
    ++p;
  } 
  return 0;
}

static int parse_args(char *buf, char *argv[])
{
  char *p;
  int argc = 0;
  p = buf;

  argv[argc++] = p;

  while (*p) {
    if ((*p == ' ') && (*(p+1) != '\0')) {
      *p = '\0';
      argv[argc++] = p+1;
    }

    if (argc >= MAX_ARGC) {
      argv[argc] = 0;
      break;
    }
    ++p;
  }

  return argc;
}

/****************************************************************************
* Name: reboot 
*
* Description:
*   Activates the watchog and resets the reader
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
static int reboot(void)
{
  /* Enable system reset watchdog and wait for reboot */
  WDTCSR |= (1 << WDE);
  for (;;) { }
  return 0;
}

/****************************************************************************
* Name: parse_cmd
*
* Description:
*   Parses incoming data from the debug terminal and interprets in as the various
*   commands that can be sent.
*
* Input Parameters:
*   data        - Single byte of data received from the debug terminal 
*
* Returned Value:
*  Always 0 
* 
* Assumptions/Limitations:
*
****************************************************************************/
static int parse_cmd(char data)
{
  static char buf[RBUF_SIZE];
  static int idx = 0;
  int putty_mode = 1;
  char *argv[MAX_ARGC+1];
  int argc = 0;
  const struct cmd *p;
  int len;
  int ret = 0;
  int process = 0;

  echo(data);
  if (putty_mode && (data == '\r'))
    echo('\n');

  if (data != '\r') {
    if ((data == ASCII_BS) || (data == ASCII_DEL)) {
      idx = idx > 0 ? idx - 1 : idx;
    } else {
      buf[idx++] = data;
    }

    if (idx >= (RBUF_SIZE-1)) {
      buf[idx] = '\0';
      len = idx;
      idx = 0;
      process = 1;
    }
  } else {
    process = 1;
    buf[idx] = '\0';
    len = idx;
    idx = 0;
  }
  
  if (process) {
    if (len != 0) {
      argc = parse_args(buf, argv);
      
      p = &commands[0];
      while (p->cmd) {
        if (strncmp(p->cmd, buf, sizeof(buf)) == 0) {
          p->callback_fn(argc, argv);
          break;
        }
        ++p;
      }

      if (!p->cmd)
        prints("Unknown command: '%s'\r\n", buf);

    }
    ret = 1;
    memset(buf, 0, sizeof(buf));
  }

  return ret; 
}

static void echo(char data)
{
  if (g_echo) {
    if (data == ASCII_DEL) /* Make backspace work */
      data = ASCII_BS;
    print_char(data);
  }
}

static void init_wd(void)
{
  /* CTC Mode, OCR0A = 250 */
  TCCR0A = (1 << WGM01);
  OCR0A = 250;

  /* Enable Output Compare interrupt */
  TIMSK0 = (1 << OCIE0A);

  /* System timer, prescaler = clk / 64 */
  TCCR0B = (1 << CS00) | (1 << CS01);
}

static void init_gpio(void)
{
#if 0
  /* Default is INPUT */
  //DDRD &= ~((1 << DDD2) | (1 << DDD1) | (1 << DDD0));

  /* Set PC7 as output for scope external trigger */
  DDRC = (1 << DDC7);

  /* Start with pin low */
  PORTC &= ~(1 << PC7);

  /* XT-1 Reset pin as output */
  DDRA = (1 << DDA0);

  /* XT-1 boot loader pin in as output */
  DDRC |= (1 << DDC5);

  /* Disable pull-ups */
  PORTD &= ~((1 << PD2) | (1 << PD1) | (1 << PD0));
  //PORTD |= ((1 << PD2) | (1 << PD1) | (1 << PD0)); /* Enable pull-up, for testing */

  /* Trigger on falling edge for D0 and D1 */
  EICRA |= (1 << ISC01) | (1 << ISC11);

  /* Any level change for CardLoad */
  EICRA |= (1 << ISC20);
  
  /* Clear any outstanding interrupts */
  EIFR = (1 << INT0) | (1 << INT1) | (1 << INT2);

  /* Interrupt enable mask */
  EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);
#endif

  /* Set ADC input as tri-stated (input, no pull-up) */
  pin_input(0, 0);

  /*  */
  pin_output(37);
  pin_low(37);

  /* For blinking the diode */
  DDRB |= _BV(DDB7);

}

static void init_usart(void)
{
  /* USART3 = Terminal */

  /* Set baud rate to 115200 */
  UBRR3H = 0;
  UBRR3L = 8;

  /* Enable TX and RX */
  UCSR3B = (1 << RXEN3) | (1 << TXEN3);

  /* Set 8N1 frame format */
  UCSR3C = (1 << USBS3) | (3 << UCSZ30);

  /* Enable RX interrupt */
  UCSR3B |= (1 << RXCIE3);

  /* USART0 = To test system */

  /* Set baud rate to 9600 */
  UBRR0H = 0;
  UBRR0L = 103; /* for 9600, required for USART0 (USB) */

  /* Enable TX and RX */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  /* Set 8N1 frame format */
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);

  /* Enable RX interrupt */
  UCSR0B |= (1 << RXCIE0);
}

int main(void)
{
  char tmp;

  /* Initialize USART rx/tx buffers */
  rbuf_init(&g_txbuf_terminal);

  /* Initalize peripherals */
  init_gpio();
  init_wd();
  init_usart();
  adc_init();

  /* Enable interrupts globally */
  sei(); 

  /* Restore the environment from EEPROM */
  prints("Restoring environment...\r\n");
  env_restore(); /* TODO: This halts execution? */

  sched_add_task(ctrl_update, 1, 500, "ctrl");

  prints("\r\nWelcome to PellShell\r\n");
  prints(">> ");
  for (;;) {
    /* Handle data from the debug terminal */
    if (rbuf_pop(&g_rxbuf_terminal, &tmp)) {
      if (parse_cmd(tmp))
        prints(">> ");
    }

    sched_runtasks();
  }
}
