#ifndef SWITCHES12_H
#define SWITCHES12_H  1

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/**
 *  Timer must be in TIMER_A_UP_MODE, not continuous
 */

#define ACLKMAX   0x7fff
//#define DEBOUNCE1  0x2a2                        /* 0.0200 with ACLK of 32K */
//#define DEBOUNCE2  0x2a2
#define DEBOUNCE1  0x2ff                        /* 0.0234 with ACLK of 32K */
// #define DEBOUNCE2  0x2ff
//#define DEBOUNCE1  0x380                        /* 0.0273 with ACLK of 32K */
//#define DEBOUNCE2  0x380
//#define DEBOUNCE1  0x666                        /* 0.0500 with ACLK of 32K */
//#define DEBOUNCE2  0x666

volatile bool sw1active;
volatile bool sw2active;

// uint_fast16_t count, ccrn;
/**
 * function prototypes
 * debounce requires timer in upmode
 */
void init_sw1sw2 (void);                    /* P1.0 LED, P1.1, P1.4 switches, INT_PORT1 */

/* start debounce timer, disable switch */
void debounce (volatile bool *switch_no, uint32_t timer,
               uint_fast16_t ccreg, uint_fast16_t ticks);

/* end debounce timer, re-enable switch */
void debounce_end (volatile bool *switch_no, uint32_t timer,
                   uint_fast16_t ccreg);

#endif
