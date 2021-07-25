/**
 *  Adaption from  guilhermelionzo /MPU9250_driverlib
 *  https://github.com/guilhermelionzo/MPU9250_driverlib.
 *
 *  Added AK8963 magnetometer functionality, temperature sensor functionality and
 *  general re-write of initialization routines adding ability to set gyro and accel
 *  sensitivity through initialization.
 *
 *  Copyright David C. Rankin,JD,PE 2021, Licence GPLv2.
 */

#include "mpu9250.h"

/**
 *  TODO create macros that take user set I2C pins from mpu9260_conf.h and stringify
 *  into GPIO_PORT_PX format replacing below.
 */

#define EUSCI_I2C_MODULE                  EUSCI_B0_BASE
#define EUSCI_I2C_PORT                    GPIO_PORT_P1
// #define EUSCI_I2C_PORT                    GPIO_PORT_P6
#define EUSCI_I2C_SCL_PIN                 GPIO_PIN7
// #define EUSCI_I2C_SCL_PIN                 GPIO_PIN5
#define EUSCI_I2C_SCL_PIN_FUNCTION        GPIO_PRIMARY_MODULE_FUNCTION
#define EUSCI_I2C_SDA_PIN                 GPIO_PIN6
// #define EUSCI_I2C_SDA_PIN                 GPIO_PIN4
#define EUSCI_I2C_SDA_PIN_FUNCTION        GPIO_PRIMARY_MODULE_FUNCTION
#define EUSCI_I2C_STATUS_SLAVE_NACK       1

/**
 *  gyro and accel sensitivity set during init from FS_SEL value provided, or
 *  set to default values of 250 deg/sec and 2 g if the default initialize function
 *  used, or in the event the user passes invalid FS_SEL values to the initialize
 *  function.
 */
float gyro_sens, accel_sens;

/**
 *  asax, asay, asaz are sensitivity adjustment data for each axis stored to fuse ROM
 *  on shipment. The adjusted value for each axis is given by:
 *
 *    Hadj = H * ( ((ASA - 128) * 0.5) / 128 + 1 )
 *
 *    where H is the asix measurement, ASA is the axis sensitivity adjustment
 *
 *  the values asax, asay, asaz hold ( ((ASA - 128) * 0.5) / 128 + 1 ) for each axis
 */
float asax, asay, asaz;

extern bool i2cinitialized;
extern bool timer32initialized;

#define RWBUFSZ  16
uint8_t buffer[RWBUFSZ];

bool readI2C (uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount);
bool writeI2C (uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount);

const uint_fast8_t MPU9250_devAddr = MPU9250_DEFAULT_ADDRESS;

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig MPU_i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

#define CLOCK_FREQ  3000000

static void delayinit (void)
{
    if (!timer32initialized)
    {
        MAP_Timer32_initModule (TIMER32_BASE, TIMER32_PRESCALER_1,
                                TIMER32_32BIT, TIMER32_PERIODIC_MODE);

        timer32initialized = true;
    }
}

/* delay in microseconds using TIMER32 */
static void delayusecs (uint32_t durationUs)
{
    delayinit();

    durationUs = durationUs * (CLOCK_FREQ / 1000000);

    MAP_Timer32_setCount (TIMER32_BASE, durationUs);
    MAP_Timer32_startTimer (TIMER32_BASE, true);

    while (MAP_Timer32_getValue (TIMER32_BASE) != 0) {}

    MAP_Timer32_haltTimer (TIMER32_BASE);
}

/**
 *  initialize I2C. The i2cinitialized variable is an external bool variable set when
 *  the first system utilizing I2C enables I2C using MAP_I2C_initMaster(). The
 *  variable prevents a complete re-initialization of I2C between peripherals, e.g.
 *  LCD, MPU, etc.. that all utilize I2C on the same line.
 */
void initI2C (void)
{
    /* I2C Clock Soruce Speed */
    //i2cConfig.i2cClk = CS_getSMCLK();

    /* Select I2C function for I2C_SCL & I2C_SDA */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin (EUSCI_I2C_PORT, EUSCI_I2C_SCL_PIN,
            EUSCI_I2C_SCL_PIN_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin (EUSCI_I2C_PORT, EUSCI_I2C_SDA_PIN,
            EUSCI_I2C_SDA_PIN_FUNCTION);

    if (!i2cinitialized)    /* if mastr not initialized, initialize here */
    {
        /* Initializing I2C Master to SMCLK at 400kbs with no-autostop */
        MAP_I2C_initMaster (EUSCI_I2C_MODULE, &MPU_i2cConfig);
        i2cinitialized = true;
    }
    // Specify slave address of MPU
    MAP_I2C_setSlaveAddress (EUSCI_I2C_MODULE, MPU9250_devAddr);

    // enable interrupts for external interrupt handlers if needed
    // MAP_I2C_enableInterrupt (EUSCI_I2C_MODULE,
    //                             EUSCI_B_I2C_START_INTERRUPT |
    //                             EUSCI_B_I2C_STOP_INTERRUPT |
    //                             EUSCI_B_I2C_NAK_INTERRUPT |
    //                             EUSCI_B_I2C_TRANSMIT_INTERRUPT0 |
    //                             EUSCI_B_I2C_RECEIVE_INTERRUPT0);

    /* Set master in transmit mode */
    MAP_I2C_setMode (EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule (EUSCI_I2C_MODULE);
}

/**
 *  Initialize mpu9250 gyro with FS_SEL = 250 and accell FS_SEL = 2
 */
void mpu9250_initialize_default (void)
{
    uint8_t data = 0x00;

    gyro_sens = 250.;
    accel_sens = 2.;

    initI2C();      /* initialize I2C */

    /* reset sample rate divisor zero  */
    writeI2C (MPU9250_devAddr, MPU9250_SMPLRT_DIV, &data, sizeof data);

    /* reset all sensors - write 0 to MPU9250_PWR_MGMT_2 */
    writeI2C (MPU9250_devAddr, MPU9250_PWR_MGMT_2, &data, sizeof data);

    /* PWR1 set - auto select best available clock */
    data = MPU9250_CLOCK_PLL_XGYRO;     /* (0x01) */
    writeI2C (MPU9250_devAddr, MPU9250_PWR_MGMT_1, &data, sizeof data);

    /* set DLPF to bandwidth 10Hz for gyro and temperature sensor */
    data = 0x05;
    writeI2C (MPU9250_devAddr, MPU9250_CONFIG, &data, sizeof data);

    /* set DLPF to bandwidth 10Hz for accel */
    writeI2C (MPU9250_devAddr, MPU9250_ACCEL_CONFIG_2, &data, sizeof data);

    /* set gyro FS_SEL */
    data = 0x00;
    writeI2C (MPU9250_devAddr, MPU9250_GYRO_CONFIG, &data, sizeof data);

    /* set accel FS_SEL */
    writeI2C (MPU9250_devAddr, MPU9250_ACCEL_CONFIG, &data, sizeof data);

}

/**
 *  Initialize mpu9250 gyro and accel with values given by gyro_fs_sel & accel_fs_sel,
 *  if parameter values do not match valid FS-SEL values the corresponding gyro or
 *  accel will be initialized to the default FS_SEL values of 250 deg/sec and 2 g.
 *
 *  TODO: add parameter to pass DLPF setting.
 */
void mpu9250_initialize (int gyro_fs_sel, int accel_fs_sel)
{
    uint8_t data = 0x00;
    uint8_t fs_sel[] = { 0x0, 0x8, 0x10, 0x18 };
    int gyro_sens_sel[] = { 250, 500, 1000, 2000 };
    int accel_sens_sel[] = { 2, 4, 8, 16 };
    int i = 0;

    gyro_sens = 250.;       /* set default gyro and access sens values */
    accel_sens = 2.;

    initI2C();              /* initialize I2C */

    /* reset sample rate divisor zero  */
    writeI2C (MPU9250_devAddr, MPU9250_SMPLRT_DIV, &data, sizeof data);

    /* reset all sensors - write 0 to MPU9250_PWR_MGMT_2 */
    writeI2C (MPU9250_devAddr, MPU9250_PWR_MGMT_2, &data, sizeof data);

    /* PWR1 set - auto select best available clock */
    data = MPU9250_CLOCK_PLL_XGYRO;     /* (0x01) */
    writeI2C (MPU9250_devAddr, MPU9250_PWR_MGMT_1, &data, sizeof data);

    /* set DLPF to bandwidth 10Hz for gyro and temperature sensor */
    data = 0x05;
    writeI2C (MPU9250_devAddr, MPU9250_CONFIG, &data, sizeof data);

    /* set DLPF to bandwidth 10Hz for accel */
    writeI2C (MPU9250_devAddr, MPU9250_ACCEL_CONFIG_2, &data, sizeof data);

    /* set gyro FS_SEL and sensitivity */
    data = 0x00;
    for (i = 0; i < sizeof fs_sel; i++)         /* iterate over parameter values */
    {
        if (gyro_fs_sel == gyro_sens_sel[i])    /* if valid parameter */
        {
            gyro_sens = gyro_fs_sel;            /* set gyro sensitivity */
            data = fs_sel[i];                   /* set gyro FS_SEL */
            break;
        }
    }
    writeI2C (MPU9250_devAddr, MPU9250_GYRO_CONFIG, &data, sizeof data);

    /* set accel FS_SEL and sensitivity */
    data = 0x00;
    for (i = 0; i < sizeof fs_sel; i++)         /* iterate over parameter values */
    {
        if (accel_fs_sel == accel_sens_sel[i])  /* if valid parameter */
        {
            accel_sens = accel_fs_sel;          /* set accel sensitivity */
            data = fs_sel[i];                   /* set accel FS_SEL */
            break;
        }
    }
    writeI2C (MPU9250_devAddr, MPU9250_ACCEL_CONFIG, &data, sizeof data);

}

/***
 *   Initialize the AK8963 magnetometer CNTL1 by placing the magnetometer in FUSE mode
 *   to read the Axis Sensitivity Adjustment (ASA) values from FUSE_ROM, power-down
 *   the chip between mode changes and then place the magnetometer in 16-bit adc and
 *   continual measurement 2 for normal operations. Save the computed adjustments
 *   factors in asax, asay, asaz. (Page 51 & 52 of Register_Map)
 */
void ak8963_initialize (void)
{
    uint8_t cntl1_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_CONT_MSRMT2,
            fuse_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_FUSE_ROM,
            reset = 0x00;

    MAP_I2C_setSlaveAddress (EUSCI_I2C_MODULE, AK8963_DEFAULT_ADDRESS);
    delayusecs (1000);

    /* set AK8963 in FUSE mode */
    writeI2C (AK8963_DEFAULT_ADDRESS, AK8963_CNTL1, &fuse_mode, sizeof fuse_mode);
    delayusecs (10000);

    /* read axis sensitivity adjustments */
    readI2C (AK8963_DEFAULT_ADDRESS, AK8963_ASAX, buffer, 3);
    delayusecs (1000);

    /* power down ak8963 before mode change */
    writeI2C (AK8963_DEFAULT_ADDRESS, AK8963_CNTL1, &reset, sizeof reset);
    delayusecs (10000);

    /* set 16-bit output and continuous measurement mode 2 */
    writeI2C (AK8963_DEFAULT_ADDRESS, AK8963_CNTL1, &cntl1_mode, sizeof cntl1_mode);
    delayusecs (1000);

    /* save sensitivity adjustment factors from the per-axis FUSE_ROM ASA as:
     *
     *  asaX = (ASA - 128) * 0.5 / 128. + 1
     */
    asax = ((buffer[0] - 128) * 0.5) / 128. + 1;
    asay = ((buffer[1] - 128) * 0.5) / 128. + 1;
    asaz = ((buffer[2] - 128) * 0.5) / 128. + 1;
}

/**
 *  Check whether MPU9250 is available on the I2C BUS
 */
bool mpu9250_checkI2C (void)
{
    uint8_t data;

    readI2C (MPU9250_devAddr, MPU9250_WHO_AM_I, &data, 1);

    if (data == MPU9250_WHO_AM_I_DATA) {
        return true;
    }

    return false;
}

/**
 *  Check whether AK8953 is available on the I2C BUS
 */
bool ak8963_checkI2C (void)
{
    uint8_t data;

    readI2C (AK8963_DEFAULT_ADDRESS, AK8963_WAI, &data, 1);

    if (data == AK8963_WAI_DATA) {
        return true;
    }

    return false;
}

/**
 *  Get raw acceleration values for linear and angular accelerations.
 */
void get_accel (int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    readI2C (MPU9250_devAddr, MPU9250_ACCEL_XOUT_H, buffer, 14);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];

    if (*ax > 32768) { *ax -= 65536; }
    if (*ay > 32768) { *ay -= 65536; }
    if (*az > 32768) { *az -= 65536; }
    if (*gx > 32768) { *gx -= 65536; }
    if (*gy > 32768) { *gy -= 65536; }
    if (*gz > 32768) { *gz -= 65536; }
}

/**
 *  Convert raw linear and angular acceleration values into linear accelerations
 *  in g and angular accelerations in deg/sec.
 */
void mpu9250_get_accel (float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz)
{
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    get_accel (&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    *ax = (raw_ax / 32768.0) * accel_sens;
    *ay = (raw_ay / 32768.0) * accel_sens;
    *az = (raw_az / 32768.0) * accel_sens;

    *gx = (raw_gx / 32768.0) * gyro_sens;
    *gy = (raw_gy / 32768.0) * gyro_sens;
    *gz = (raw_gz / 32768.0) * gyro_sens;

}

/**
 *  Get raw values for all accelerations plus temperature.
 */
void get_accel_w_temp (int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz,
                       int16_t *tempc)
{
    readI2C (MPU9250_devAddr, MPU9250_ACCEL_XOUT_H, buffer, 14);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];

    *tempc = (((int16_t)buffer[6]) << 8) | buffer[7];

    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];

    if (*ax > 32768) { *ax -= 65536; }
    if (*ay > 32768) { *ay -= 65536; }
    if (*az > 32768) { *az -= 65536; }

    if (*tempc > 32768) { *tempc -= 65536; }

    if (*gx > 32768) { *gx -= 65536; }
    if (*gy > 32768) { *gy -= 65536; }
    if (*gz > 32768) { *gz -= 65536; }
}

/**
 *  Convert values for all accelerations to g, deg/sec and temperature in deg C.
 */
void mpu9250_get_accel_w_temp (float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *tempc)
{
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz, raw_temp;
    float room_temp_offset = 0.;

    get_accel_w_temp (&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz, &raw_temp);

    *ax = (raw_ax / 32768.0) * accel_sens;
    *ay = (raw_ay / 32768.0) * accel_sens;
    *az = (raw_az / 32768.0) * accel_sens;

    *tempc = (raw_temp - room_temp_offset) / 333.87 + 21.;

    *gx = (raw_gx / 32768.0) * gyro_sens;
    *gy = (raw_gy / 32768.0) * gyro_sens;
    *gz = (raw_gz / 32768.0) * gyro_sens;
}

/**
 *  Get individual accelerations, temp and magnetometer readings. Functions are ordered
 *  in the order of their respective register addresses, e.g.
 *
 *  Accelerometer, Temperature, Gyro, then Magnetometer on AK8963 chip.
 */

/**
 *  Get raw linear accelerations values.
 */
void get_lin_accel (int16_t *ax, int16_t *ay, int16_t *az)
{
    readI2C (MPU9250_devAddr, MPU9250_ACCEL_XOUT_H, buffer, 6);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];

    if (*ax > 32768) { *ax -= 65536; }
    if (*ay > 32768) { *ay -= 65536; }
    if (*az > 32768) { *az -= 65536; }
}

/**
 *  Convert raw linear acceleration values to g.
 */
void mpu9250_aet_lin_accel (float *ax, float *ay, float *az)
{
    int16_t raw_ax, raw_ay, raw_az;

    get_lin_accel (&raw_ax, &raw_ay, &raw_az);

    *ax = (raw_ax / 32768.0) * accel_sens;
    *ay = (raw_ay / 32768.0) * accel_sens;
    *az = (raw_az / 32768.0) * accel_sens;
}

/**
 *  Get raw temperature value.
 */
void get_tempc (int16_t *tempc)
{
    readI2C (MPU9250_devAddr, MPU9250_TEMP_OUT_H, buffer, 2);

    *tempc = (((int16_t)buffer[0]) << 8) | buffer[1];

    if (*tempc > 32768) { *tempc -= 65536; }
}

/**
 *  Convert raw temperature value to deg C.
 */
void mpu9250_get_tempc (float *tempc)
{
    int16_t raw_temp;
    float room_temp_offset = 0.;

    get_tempc (&raw_temp);

    *tempc = (raw_temp - room_temp_offset) / 333.87 + 21.;
}

/**
 *  Get raw angular acceleration values from gyro.
 */
void get_ang_accel (int16_t *gx, int16_t *gy, int16_t *gz)
{
    readI2C (MPU9250_devAddr, MPU9250_GYRO_XOUT_H, buffer, 6);

    *gx = (((int16_t)buffer[0]) << 8) | buffer[1];
    *gy = (((int16_t)buffer[2]) << 8) | buffer[3];
    *gz = (((int16_t)buffer[4]) << 8) | buffer[5];

    if (*gx > 32768) { *gx -= 65536; }
    if (*gy > 32768) { *gy -= 65536; }
    if (*gz > 32768) { *gz -= 65536; }
}

/**
 * Convert raw angular acceleration values to deg/sec.
 */
void mpu9250_get_ang_accel (float *gx, float *gy, float *gz)
{
    int16_t raw_gx, raw_gy, raw_gz;

    get_ang_accel (&raw_gx, &raw_gy, &raw_gz);

    *gx = (raw_gx / 32768.0) * gyro_sens;
    *gy = (raw_gy / 32768.0) * gyro_sens;
    *gz = (raw_gz / 32768.0) * gyro_sens;
}

/**
 *  Get raw magnetometer orientation values
 */
void get_orientation (int16_t *mx, int16_t *my, int16_t *mz)
{
    readI2C (AK8963_DEFAULT_ADDRESS, AK8963_STI, buffer, 8);

    *mx = buffer[1] | (buffer[2] << 8);
    *my = buffer[3] | (buffer[4] << 8);
    *mz = buffer[5] | (buffer[6] << 8);

    if (*mx > 32768) { *mx -= 65536; }
    if (*my > 32768) { *my -= 65536; }
    if (*mz > 32768) { *mz -= 65536; }
}

/**
 *  Convert raw magnetometer values into mx, my, mz in uT
 */
void ak8963_get_orientation (float *mx, float *my, float *mz)
{
    int16_t mx_raw, my_raw, mz_raw;
    float mag_sens = 4912.0;            /* magnetometer sensitivity: 4800 uT */

    get_orientation (&mx_raw, &my_raw, &mz_raw);

    /* convert to uT */
    *mx = (mx_raw / 32768.0) * mag_sens * asaz;
    *my = (my_raw / 32768.0) * mag_sens * asay;
    *mz = (mz_raw / 32768.0) * mag_sens * asaz;
}

/**
 *  write to slave address and register from Data, ui8ByteCount bytes
 */
bool writeI2C (uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount)
{
    /* Todo: Implement a delay */
    /* Wait until ready to write */
    while (MAP_I2C_isBusBusy(EUSCI_I2C_MODULE));

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_I2C_MODULE, ui8Addr);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_I2C_MODULE, ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Now write one or more data bytes */
    while(1)
    {
        /* Wait for next INT */
        while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

        /* If no data to follow, we are done */
        if(ui8ByteCount == 0 )
        {
            MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
            MAP_I2C_clearInterruptFlag(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
            return(true);
        }
        /* If more, send the next byte */
        else
        {
            MAP_I2C_masterSendMultiByteNext(EUSCI_I2C_MODULE, *Data++);
        }
        ui8ByteCount--;
    }
}

/**
 *  read from slave address and register into Data, ui8ByteCount bytes
 */
bool readI2C (uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint8_t ui8ByteCount)
{
    /* Wait until ready */
    while (MAP_I2C_isBusBusy(EUSCI_I2C_MODULE)) {}

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_I2C_MODULE, ui8Addr);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_I2C_MODULE,ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) &
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {}

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Turn off TX and generate RE-Start */
    MAP_I2C_masterReceiveStart(EUSCI_I2C_MODULE);

    /* Wait for start bit to complete */
    while (MAP_I2C_masterIsStartSent(EUSCI_I2C_MODULE)) {}

    if((MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_NAK_INTERRUPT)) &
            EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_I2C_MODULE);
        return(EUSCI_I2C_STATUS_SLAVE_NACK);
    }

    /* Read one or more bytes */
    while(ui8ByteCount)
    {
        /* If reading 1 byte (or last byte), generate the stop to meet the spec */
        if(ui8ByteCount-- == 1)
        {
            *Data++ = MAP_I2C_masterReceiveMultiByteFinish (EUSCI_I2C_MODULE);
        }
        else
        {
            /* Wait for next RX interrupt */
            while(!(MAP_I2C_getInterruptStatus(EUSCI_I2C_MODULE, EUSCI_B_I2C_RECEIVE_INTERRUPT0) &
                    EUSCI_B_I2C_RECEIVE_INTERRUPT0)) {}

            /* Read the rx byte */
            *Data++ = MAP_I2C_masterReceiveMultiByteNext (EUSCI_I2C_MODULE);
        }
    }

    MAP_I2C_clearInterruptFlag (EUSCI_I2C_MODULE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);

    return (true);
}

/**
 *  write less than 1 byte to register preserving bits not written.
 */
void writeBits (uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b = 0;

    readI2C (MPU9250_devAddr, regAddr, &b, 1);

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);

    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    writeI2C (MPU9250_devAddr, regAddr, &b, 1);
}

/**
 *  write single bitNum bit to register from data, preserving all other bits.
 */
void writeBit (uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b;

    readI2C (MPU9250_devAddr,regAddr, &b, 1);

    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    writeI2C (MPU9250_devAddr, regAddr, &b, 1);

}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_SLEEP_BIT
 */
void setSleepEnabled (bool enabled)
{
    writeBit (MPU9250_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void setFullScaleAccelRange (uint8_t range)
{
    writeBits (MPU9250_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT,
                MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU9250_GYRO_FS_250
 * @see MPU9250_RA_GYRO_CONFIG
 * @see MPU9250_GCONFIG_FS_SEL_BIT
 * @see MPU9250_GCONFIG_FS_SEL_LENGTH
 */
void setFullScaleGyroRange (uint8_t range)
{
    writeBits (MPU9250_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT,
                MPU9250_GCONFIG_FS_SEL_LENGTH, range);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_CLKSEL_BIT
 * @see MPU9250_PWR1_CLKSEL_LENGTH
*/
void setClockSource (uint8_t source)
{
    writeBits (MPU9250_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT,
                MPU9250_PWR1_CLKSEL_LENGTH, source);
}

