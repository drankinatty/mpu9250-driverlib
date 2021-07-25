/**
 * MSP432 MPU9250 Demonstration Program
 *
 * Description: Example Program showing MPU9250 implementation over I2C for the MSP432
 *              using DriverLib. Hardware connection is simle. Pin 1.6 provides data
 *              (SDA) and Pin 1.7 provides the clock (SCL). The MPU9250 requires that
 *              SCL and ECL are wired together and SDA and EDA must also be wired
 *              together due to the magnetomet AK8963 chip being on the extended I2C
 *              bus. A 3.3v source and ground connect to VCC and GND, respectively.
 *
 *              On startup the example validates initialization of both the MPU9250 and
 *              AK8963 chips. It also validates I2C communication to each before
 *              displaying the MPU sensor data. Switch1 (P1.1) starts automatic update
 *              of data from the sensor once per-second and Switch2 (P1.4) stops the
 *              automatc update.
 *
 *
 *                       MSP432P401
 *
 *                     PWR          RST
 *                   __---___________v___
 *                  |                    |
 *  start           |                    |    stop                 MPU9250
 *  automatic    -->| Sw1            Sw2 |<-- data update       +-----------+
 *  data update     |                    |                      | VCC       |
 *                  |                ... |                      | GND       |
 *                  |                o o-|-- 1.6 sda--     /---+| SCL----+  |
 *                  |                o o-|-- 1.7 scl  \--------+| SDA-+  |  |
 *                  |                ... |         \-----/      | EDA-+  |  |
 *                  |                    |                      | ECL----+  |
 *                  |                    |                      | ...       |
 *                  |                  o |
 *                  |                  o-|-- gnd  to GND     note: SCL and ECL must
 *                  |                  o-|-- 3.3v to VCC           be connected, and
 *                                                                 SDA and EDA must
 *                                                                 be connected.
 *                                                          AK8963 is on extended I2C bus
 *
 *  Copyright David C. Rankin,JD,PE 2021, Licence GPLv2.
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "eusci_A_uart.h"           /* EUSCIA0 initialization */
#include "mpu9250.h"                /* MPU-9250 library using i2c */
#include "numeric_str_cnv.h"        /* quick and dirty int/float to str functions */
#include "switches.h"               /* button 1, 2, debounce and LED 1 init */

#define MPU9250_devAddr MPU9250_DEFAULT_ADDRESS

float ax, ay, az,                           /* variables holding MPU data */
      gx, gy, gz,
      mx, my, mz,
      tempc;

static volatile bool updateMPUdata;         /* flag - update data once per-second */
static volatile bool btn1pressed;           /* flag - button1 (P1.1) pressed */
static volatile bool btn2pressed;           /* flag - button2 (P1.4) pressed */

static volatile uint_fast8_t amPowerState;  /* saved active-mode power state */

bool i2cinitialized;                        /* flags for extern use indicating I2C */
bool timer32initialized;                    /* and Timer32 initialized */

/* default - 3MHz SMCLK for eUSCIA/UART at 115200 baud */
extern const eUSCI_UART_ConfigV1 uartConfig;

/**
 *  TimerA1 UpMode Configuration Parameter,
 *  timer used for data update and switch debounce
 */
const Timer_A_UpModeConfig upConfigTA1 =
{
  TIMER_A_CLOCKSOURCE_ACLK,             /* ACLK Clock 32 KHz  (uint_fast16_t clockSource) */
  TIMER_A_CLOCKSOURCE_DIVIDER_1,        /* Rollover in 4 sec  (uint_fast16_t clockSourceDivider) */
  ACLKMAX,                              /* 32767 ticks        (uint_fast16_t timerPeriod) */
  TIMER_A_TAIE_INTERRUPT_DISABLE,       /* Rollover TAIE      (uint_fast16_t timerInterruptEnable_TAIE) */
  TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,   /* CCR0 CCR0IE        (uint_fast16_t captureCompareInterruptEnable_CCR0_CCIE) */
  TIMER_A_DO_CLEAR                      /* Clear Timer        (uint_fast16_t timerClear) */
};

/* function prototypes */
void update_MPU_data (void);
void btn1_handler (void);
void btn2_handler (void);
void checkI2C (void);

int main (void)
{
    MAP_WDT_A_holdTimer();                          /* Stop Watchdog  */

    init_sw1sw2();                                  /* init switch1 & 2 and LED0 */

    init_UART (EUSCI_A0_BASE, &uartConfig);         /* init EUSCI_A0 for uart */

    mpu9250_initialize_default();                   /* initialize MPU9250 accelerometer */
    uart_tx_str (EUSCI_A0_BASE, "MPU9250: initialized\n");

    ak8963_initialize();                            /* initialize AK8963 magnetometer */
    uart_tx_str (EUSCI_A0_BASE, "AK8963 : initialized\n");

    checkI2C();                                     /* check I2C for MPU9250 & AK8963 */

    /* Configuring TimerA1 for Up Mode for data update and debounce timer */
    MAP_Timer_A_configureUpMode (TIMER_A1_BASE, &upConfigTA1);

    /* Enable TimerA1 CCR0 interrupt for updates from MPU and CCR1 for switch debounce */
    MAP_Interrupt_enableInterrupt (INT_TA1_0);
    MAP_Interrupt_enableInterrupt (INT_TA1_N);

    MAP_Interrupt_enableMaster();           /* enable processing all interrupts */

    /* Starting the Timer_A1 in up mode */
    MAP_Timer_A_startCounter (TIMER_A1_BASE, TIMER_A_UP_MODE);

    amPowerState = MAP_PCM_getPowerState();       /* aave active mode power state */

    /* output instructions */
    uart_tx_str (EUSCI_A0_BASE, "\nUsage:\n  switch1 (P1.1) - starts data update\n"
                 "  switch2 (P1.4) - stops data update\n");

    /* output MPU data heading */
    uart_tx_str (EUSCI_A0_BASE, "\nAccelerations (linear/angular), Magnetic Field, "
                 "and Temperature\n");

    /* output MPU data */
    update_MPU_data();

    /* program loop processes in active-mode, then sleeps in LMP3 until active-mode
     * restored by interrupt.
     */
    while (true)
    {
        if (btn1pressed) {          /* respond to button1 (P1.1) press in active-mode */
            btn1_handler();         /* sets updateMPUdata true */
        }
        if (btn2pressed) {          /* respond to button2 (P1.4) press in active-mode */
            btn2_handler();         /* sets updateMPUdata false */
        }
        if (updateMPUdata) {        /* if flag set, update all values */
            update_MPU_data();
        }

        MAP_Interrupt_enableSleepOnIsrExit();   /* sleep after processing all data */
        MAP_PCM_gotoLPM3();                     /* transition to LMP3 */
    }
}

/*
 * Port 1 ISR
 */
void PORT1_IRQHandler (void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus (GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag (GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);

    if (status & GPIO_PIN1 && sw1active) {  /* respond to Pin1 Interrupt (switch 1) */
        debounce (&sw1active, TIMER_A1_BASE,
                  TIMER_A_CAPTURECOMPARE_REGISTER_1, DEBOUNCE1);

        btn1pressed = true;
    }

    if (status & GPIO_PIN4 && sw2active) {  /* respond to Pin4 Interrupt (switch 2) */
        debounce (&sw2active, TIMER_A1_BASE,
                  TIMER_A_CAPTURECOMPARE_REGISTER_2, DEBOUNCE1);

        btn2pressed = true;
    }

    MAP_Interrupt_disableSleepOnIsrExit();
    MAP_PCM_setPowerState (amPowerState);
}

/**
 * TimerA_1 CCR0 Interrupt, restore active power mode to procecc data display
 */
void TA1_0_IRQHandler(void)
{
    MAP_Timer_A_clearCaptureCompareInterrupt (TIMER_A1_BASE,
                                              TIMER_A_CAPTURECOMPARE_REGISTER_0);

    if (updateMPUdata)
    {
        MAP_Interrupt_disableSleepOnIsrExit();
        MAP_PCM_setPowerState (amPowerState);
    }
}

/**
 * TimerA_1 CCRn, debounce switch presses
 */
void TA1_N_IRQHandler(void)
{
    /* debounce button 1 interrupt */
    debounce_end (&sw1active, TIMER_A1_BASE,
                  TIMER_A_CAPTURECOMPARE_REGISTER_1);

    /* debounce button 2 interrupt */
    debounce_end (&sw2active, TIMER_A1_BASE,
                  TIMER_A_CAPTURECOMPARE_REGISTER_2);
}

void update_MPU_data (void)
{
    char str[FPMAXC] = "";
    float tempf;

    /* retrieve linear, angular and temp from MPU9250 */
    mpu9250_get_accel_w_temp (&ax, &ay, &az, &gx, &gy, &gz, &tempc);

    tempf = tempc * 9. / 5. + 32.;                  /* compute Fahrenheight temp */

    /* retrieve magnetometer readings from AK8963 */
    ak8963_get_orientation (&mx, &my, &mz);

    /* display linear acceleration */
    float2strw (str, 8, ax, 2);
    uart_tx_str (EUSCI_A0_BASE, "\n  accel: {");
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, ay, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, az, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, "  }\n  gyro : {");

    /* display angular acceleration from gyro */
    float2strw (str, 8, gx, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, gy, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, gz, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, "  }\n");

    /* display magnetometer field density */
    uart_tx_str (EUSCI_A0_BASE, "  mag  : {");
    float2strw (str, 8, mx, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, my, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, ", ");
    float2strw (str, 8, mz, 2);
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, "  }\n");

    /* display temperature data */
    float2strw (str, 8, tempc, 2);
    uart_tx_str (EUSCI_A0_BASE, "  temp : {");
    uart_tx_str (EUSCI_A0_BASE, str);

    float2strw (str, 8, tempf, 2);
    uart_tx_str (EUSCI_A0_BASE, " C / ");
    uart_tx_str (EUSCI_A0_BASE, str);
    uart_tx_str (EUSCI_A0_BASE, " F  }\n");
}

void btn1_handler (void)
{
    updateMPUdata = true;
    uart_tx_str (EUSCI_A0_BASE, "\nAutomatic data update started:\n");

    btn1pressed = !btn1pressed;
}

void btn2_handler (void)
{
    updateMPUdata = false;
    uart_tx_str (EUSCI_A0_BASE, "\nAutomatic data update stopped.\n");

    btn2pressed = !btn2pressed;
}

/**
 *  Check I2C connections with MPU9250 and 3rd-party slave AK8963 magnetometer.
 */
void checkI2C (void)
{
    /* validate I2C connection with MPU9250 */
    if (mpu9250_checkI2C()) {
        uart_tx_str (EUSCI_A0_BASE, "\nMPU9250 I2C: connection established.\n");
    }
    else {
        uart_tx_str (EUSCI_A0_BASE, "\nerror: MPU9250 I2C connection failed.\n\n");
    }

    /* validate I2C connection with AK8953 */
    if (ak8963_checkI2C()) {
        uart_tx_str (EUSCI_A0_BASE, "AK8963 I2C : connection established.\n");
    }
    else {
        uart_tx_str (EUSCI_A0_BASE, "\nerror: AK8963 I2C connection failed.\n\n");
    }
}

