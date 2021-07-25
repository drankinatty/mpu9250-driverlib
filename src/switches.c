#include "switches.h"

/** enable P1.1 & P1.4 Switch1 and Switch2 with pullup resistor, enable P1.0 LED,
 *  and endable INT_PORT1 interrupt.
 */
void init_sw1sw2 (void)
{
    sw1active = sw2active = true;           /* switches initialized active */

    /* Configuring P1.0 as output (trend temp increasing - on / decreasing - blinking) */
    MAP_GPIO_setAsOutputPin (GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin (GPIO_PORT_P1, GPIO_PIN0);

   /* Configure P1.1 (switch_1) and P1.4 (switch2) as input with pull-up,
    * set edge-select to trigger on high-to-low transition
    */
    MAP_GPIO_clearInterruptFlag (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_interruptEdgeSelect (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4,
                                  GPIO_HIGH_TO_LOW_TRANSITION);

    /* Configure Port1 Interrupt for both Pin1 and Pin4 and enable interrupt */
    MAP_GPIO_enableInterrupt (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_Interrupt_enableInterrupt (INT_PORT1);
}

/** start debounce timer, disable switch for ticks timer ticks */
void debounce (volatile bool *switch_no, uint32_t timer,
               uint_fast16_t ccreg, uint_fast16_t ticks)
{
    uint_fast16_t count, ccrn;
//    uint_fast16_t count = MAP_Timer_A_getCounterValue (timer),
//                  ccrn = ticks + count;
    count = MAP_Timer_A_getCounterValue (timer);    /* unreliable value (see manual) */

    if (count > ACLKMAX)                            /* limit to range of ACLKMAX */
    {
        count = ACLKMAX;
    }

    ccrn = ticks + count;

    if (ccrn > ACLKMAX) {
        ccrn -= ACLKMAX;
    }

    /* disable button presses */
    *switch_no = false;

    /* Set Capture/Compre Register 1 */
    MAP_Timer_A_setCompareValue (timer, ccreg, ccrn);

    /* enable CCR1 compare interrupt */
    MAP_Timer_A_enableCaptureCompareInterrupt (timer, ccreg);
}

/** end debounce timer, re-enable switch */
void debounce_end (volatile bool *switch_no, uint32_t timer,
                   uint_fast16_t ccreg)
{
    /* debounce button 1 interrupt */
    if (MAP_Timer_A_getCaptureCompareInterruptStatus (timer, ccreg,
                                      TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG)) {

        MAP_Timer_A_clearCaptureCompareInterrupt (timer, ccreg);

        /* enable CCR1 compare interrupt */
        MAP_Timer_A_disableCaptureCompareInterrupt (timer, ccreg);

        *switch_no = true;
    }
}

