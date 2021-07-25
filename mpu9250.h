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

#ifndef MPU9250_H_
#define MPU9250_H_  1

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdbool.h>
#include <stdint.h>

#define MPU9250_ADDRESS_AD0_LOW             0x68 // address pin low (GND), default InvenSense eval board
#define MPU9250_ADDRESS_AD0_HIGH            0x69 // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS             MPU9250_ADDRESS_AD0_LOW

#define MPU9250_SELF_TEST_X_GYRO            0x00
#define MPU9250_SELF_TEST_Y_GYRO            0x01
#define MPU9250_SELF_TEST_Z_GYRO            0x02
#define MPU9250_SELF_TEST_X_ACCEL           0x0D
#define MPU9250_SELF_TEST_Y_ACCEL           0x0E
#define MPU9250_SELF_TEST_Z_ACCEL           0x0F

#define MPU9250_XG_OFFSET_H                 0x13
#define MPU9250_XG_OFFSET_L                 0x14
#define MPU9250_YG_OFFSET_H                 0x15
#define MPU9250_YG_OFFSET_L                 0x16
#define MPU9250_ZG_OFFSET_H                 0x17
#define MPU9250_ZG_OFFSET_L                 0x18

#define MPU9250_SMPLRT_DIV                  0x19

#define MPU9250_CONFIG                      0x1A
#define MPU9250_GYRO_CONFIG                 0x1B
#define MPU9250_ACCEL_CONFIG                0x1C
#define MPU9250_ACCEL_CONFIG_2              0x1D

#define MPU9250_LP_ACCEL_ODR                0x1E
#define MPU9250_WOM_THR                     0x1F
#define MPU9250_FIFO_EN                     0x23

#define MPU9250_I2C_MST_CTRL                0x24
#define MPU9250_I2C_SLV0_ADDR               0x25
#define MPU9250_I2C_SLV0_REG                0x26
#define MPU9250_I2C_SLV0_CTRL               0x27
#define MPU9250_I2C_SLV1_ADDR               0x28
#define MPU9250_I2C_SLV1_REG                0x29
#define MPU9250_I2C_SLV1_CTRL               0x2A
#define MPU9250_I2C_SLV2_ADDR               0x2B
#define MPU9250_I2C_SLV2_REG                0x2C
#define MPU9250_I2C_SLV2_CTRL               0x2D
#define MPU9250_I2C_SLV3_ADDR               0x2E
#define MPU9250_I2C_SLV3_REG                0x2F
#define MPU9250_I2C_SLV3_CTRL               0x30
#define MPU9250_I2C_SLV4_ADDR               0x31
#define MPU9250_I2C_SLV4_REG                0x32
#define MPU9250_I2C_SLV4_DO                 0x33
#define MPU9250_I2C_SLV4_CTRL               0x34
#define MPU9250_I2C_SLV4_DI                 0x35
#define MPU9250_I2C_MST_STATUS              0x36

#define MPU9250_INT_PIN_CFG                 0x37
#define MPU9250_INT_ENABLE                  0x38
#define MPU9250_INT_STATUS                  0x3A

#define MPU9250_ACCEL_XOUT_H                0x3B
#define MPU9250_ACCEL_XOUT_L                0x3C
#define MPU9250_ACCEL_YOUT_H                0x3D
#define MPU9250_ACCEL_YOUT_L                0x3E
#define MPU9250_ACCEL_ZOUT_H                0x3F
#define MPU9250_ACCEL_ZOUT_L                0x40

#define MPU9250_TEMP_OUT_H                  0x41
#define MPU9250_TEMP_OUT_L                  0x42

#define MPU9250_GYRO_XOUT_H                 0x43
#define MPU9250_GYRO_XOUT_L                 0x44
#define MPU9250_GYRO_YOUT_H                 0x45
#define MPU9250_GYRO_YOUT_L                 0x46
#define MPU9250_GYRO_ZOUT_H                 0x47
#define MPU9250_GYRO_ZOUT_L                 0x48

#define MPU9250_EXT_SENS_DATA_00            0x49
#define MPU9250_EXT_SENS_DATA_01            0x4A
#define MPU9250_EXT_SENS_DATA_02            0x4B
#define MPU9250_EXT_SENS_DATA_03            0x4C
#define MPU9250_EXT_SENS_DATA_04            0x4D
#define MPU9250_EXT_SENS_DATA_05            0x4E
#define MPU9250_EXT_SENS_DATA_06            0x4F
#define MPU9250_EXT_SENS_DATA_07            0x50
#define MPU9250_EXT_SENS_DATA_08            0x51
#define MPU9250_EXT_SENS_DATA_09            0x52
#define MPU9250_EXT_SENS_DATA_10            0x53
#define MPU9250_EXT_SENS_DATA_11            0x54
#define MPU9250_EXT_SENS_DATA_12            0x55
#define MPU9250_EXT_SENS_DATA_13            0x56
#define MPU9250_EXT_SENS_DATA_14            0x57
#define MPU9250_EXT_SENS_DATA_15            0x58
#define MPU9250_EXT_SENS_DATA_16            0x59
#define MPU9250_EXT_SENS_DATA_17            0x5A
#define MPU9250_EXT_SENS_DATA_18            0x5B
#define MPU9250_EXT_SENS_DATA_19            0x5C
#define MPU9250_EXT_SENS_DATA_20            0x5D
#define MPU9250_EXT_SENS_DATA_21            0x5E
#define MPU9250_EXT_SENS_DATA_22            0x5F
#define MPU9250_EXT_SENS_DATA_23            0x60

#define MPU9250_I2C_SLV0_DO                 0x63
#define MPU9250_I2C_SLV1_DO                 0x64
#define MPU9250_I2C_SLV2_DO                 0x65
#define MPU9250_I2C_SLV3_DO                 0x66
#define MPU9250_I2C_MST_DELAY_CTRL          0x67

#define MPU9250_SIGNAL_PATH_RESET           0x68
#define MPU9250_MOT_DETECT_CTRL             0x69
#define MPU9250_USER_CTRL                   0x6A

#define MPU9250_PWR_MGMT_1                  0x6B
#define MPU9250_PWR_MGMT_2                  0x6C

#define MPU9250_BANK_SEL                    0x6D
#define MPU9250_MEM_START_ADDR              0x6E
#define MPU9250_MEM_R_W                     0x6F
#define MPU9250_DMP_CFG_1                   0x70
#define MPU9250_DMP_CFG_2                   0x71

#define MPU9250_FIFO_COUNTH                 0x72
#define MPU9250_FIFO_COUNTL                 0x73
#define MPU9250_FIFO_R_W                    0x74

#define MPU9250_WHO_AM_I                    0x75
#define MPU9250_WHO_AM_I_DATA          0x71

#define MPU9250_XA_OFFSET_H                 0x77
#define MPU9250_XA_OFFSET_l                 0x78
#define MPU9250_YA_OFFSET_H                 0x7A
#define MPU9250_YA_OFFSET_l                 0x7B
#define MPU9250_ZA_OFFSET_H                 0x7D
#define MPU9250_ZA_OFFSET_l                 0x7E

#define MPU9250_TC_PWR_MODE_BIT               7
#define MPU9250_TC_OFFSET_BIT                 6
#define MPU9250_TC_OFFSET_LENGTH              6
#define MPU9250_TC_OTP_BNK_VLD_BIT            0

#define MPU9250_VDDIO_LEVEL_VLOGIC            0
#define MPU9250_VDDIO_LEVEL_VDD               1

#define MPU9250_CFG_EXT_SYNC_SET_BIT          5
#define MPU9250_CFG_EXT_SYNC_SET_LENGTH       3
#define MPU9250_CFG_DLPF_CFG_BIT              2
#define MPU9250_CFG_DLPF_CFG_LENGTH           3

#define MPU9250_EXT_SYNC_DISABLED           0x0
#define MPU9250_EXT_SYNC_TEMP_OUT_L         0x1
#define MPU9250_EXT_SYNC_GYRO_XOUT_L        0x2
#define MPU9250_EXT_SYNC_GYRO_YOUT_L        0x3
#define MPU9250_EXT_SYNC_GYRO_ZOUT_L        0x4
#define MPU9250_EXT_SYNC_ACCEL_XOUT_L       0x5
#define MPU9250_EXT_SYNC_ACCEL_YOUT_L       0x6
#define MPU9250_EXT_SYNC_ACCEL_ZOUT_L       0x7

#define MPU9250_DLPF_BW_256                 0x00
#define MPU9250_DLPF_BW_188                 0x01
#define MPU9250_DLPF_BW_98                  0x02
#define MPU9250_DLPF_BW_42                  0x03
#define MPU9250_DLPF_BW_20                  0x04
#define MPU9250_DLPF_BW_10                  0x05
#define MPU9250_DLPF_BW_5                   0x06

#define MPU9250_GCONFIG_FS_SEL_BIT              4
#define MPU9250_GCONFIG_FS_SEL_LENGTH           2

#define MPU9250_GYRO_FS_250                 0x00
#define MPU9250_GYRO_FS_500                 0x01
#define MPU9250_GYRO_FS_1000                0x02
#define MPU9250_GYRO_FS_2000                0x03

#define MPU9250_ACONFIG_XA_ST_BIT               7
#define MPU9250_ACONFIG_YA_ST_BIT               6
#define MPU9250_ACONFIG_ZA_ST_BIT               5
#define MPU9250_ACONFIG_AFS_SEL_BIT             4
#define MPU9250_ACONFIG_AFS_SEL_LENGTH          2
#define MPU9250_ACONFIG_ACCEL_HPF_BIT           2
#define MPU9250_ACONFIG_ACCEL_HPF_LENGTH        3

#define MPU9250_ACCEL_FS_2                  0x00
#define MPU9250_ACCEL_FS_4                  0x01
#define MPU9250_ACCEL_FS_8                  0x02
#define MPU9250_ACCEL_FS_16                 0x03

#define MPU9250_DHPF_RESET                  0x00
#define MPU9250_DHPF_5                      0x01
#define MPU9250_DHPF_2P5                    0x02
#define MPU9250_DHPF_1P25                   0x03
#define MPU9250_DHPF_0P63                   0x04
#define MPU9250_DHPF_HOLD                   0x07

#define MPU9250_TEMP_FIFO_EN_BIT                7
#define MPU9250_XG_FIFO_EN_BIT                  6
#define MPU9250_YG_FIFO_EN_BIT                  5
#define MPU9250_ZG_FIFO_EN_BIT                  4
#define MPU9250_ACCEL_FIFO_EN_BIT               3
#define MPU9250_SLV2_FIFO_EN_BIT                2
#define MPU9250_SLV1_FIFO_EN_BIT                1
#define MPU9250_SLV0_FIFO_EN_BIT                0

#define MPU9250_MULT_MST_EN_BIT                 7
#define MPU9250_WAIT_FOR_ES_BIT                 6
#define MPU9250_SLV_3_FIFO_EN_BIT               5
#define MPU9250_I2C_MST_P_NSR_BIT               4
#define MPU9250_I2C_MST_CLK_BIT                 3
#define MPU9250_I2C_MST_CLK_LENGTH              4

#define MPU9250_CLOCK_DIV_348                 0x0
#define MPU9250_CLOCK_DIV_333                 0x1
#define MPU9250_CLOCK_DIV_320                 0x2
#define MPU9250_CLOCK_DIV_308                 0x3
#define MPU9250_CLOCK_DIV_296                 0x4
#define MPU9250_CLOCK_DIV_286                 0x5
#define MPU9250_CLOCK_DIV_276                 0x6
#define MPU9250_CLOCK_DIV_267                 0x7
#define MPU9250_CLOCK_DIV_258                 0x8
#define MPU9250_CLOCK_DIV_500                 0x9
#define MPU9250_CLOCK_DIV_471                 0xA
#define MPU9250_CLOCK_DIV_444                 0xB
#define MPU9250_CLOCK_DIV_421                 0xC
#define MPU9250_CLOCK_DIV_400                 0xD
#define MPU9250_CLOCK_DIV_381                 0xE
#define MPU9250_CLOCK_DIV_364                 0xF

#define MPU9250_I2C_SLV_RW_BIT                  7
#define MPU9250_I2C_SLV_ADDR_BIT                6
#define MPU9250_I2C_SLV_ADDR_LENGTH             7
#define MPU9250_I2C_SLV_EN_BIT                  7
#define MPU9250_I2C_SLV_BYTE_SW_BIT             6
#define MPU9250_I2C_SLV_REG_DIS_BIT             5
#define MPU9250_I2C_SLV_GRP_BIT                 4
#define MPU9250_I2C_SLV_LEN_BIT                 3
#define MPU9250_I2C_SLV_LEN_LENGTH              4

#define MPU9250_I2C_SLV4_RW_BIT                 7
#define MPU9250_I2C_SLV4_ADDR_BIT               6
#define MPU9250_I2C_SLV4_ADDR_LENGTH            7
#define MPU9250_I2C_SLV4_EN_BIT                 7
#define MPU9250_I2C_SLV4_INT_EN_BIT             6
#define MPU9250_I2C_SLV4_REG_DIS_BIT            5
#define MPU9250_I2C_SLV4_MST_DLY_BIT            4
#define MPU9250_I2C_SLV4_MST_DLY_LENGTH         5

#define MPU9250_MST_PASS_THROUGH_BIT            7
#define MPU9250_MST_I2C_SLV4_DONE_BIT           6
#define MPU9250_MST_I2C_LOST_ARB_BIT            5
#define MPU9250_MST_I2C_SLV4_NACK_BIT           4
#define MPU9250_MST_I2C_SLV3_NACK_BIT           3
#define MPU9250_MST_I2C_SLV2_NACK_BIT           2
#define MPU9250_MST_I2C_SLV1_NACK_BIT           1
#define MPU9250_MST_I2C_SLV0_NACK_BIT           0

#define MPU9250_INTCFG_INT_LEVEL_BIT            7
#define MPU9250_INTCFG_INT_OPEN_BIT             6
#define MPU9250_INTCFG_LATCH_INT_EN_BIT         5
#define MPU9250_INTCFG_INT_RD_CLEAR_BIT         4
#define MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT      3
#define MPU9250_INTCFG_FSYNC_INT_EN_BIT         2
#define MPU9250_INTCFG_I2C_BYPASS_EN_BIT        1
#define MPU9250_INTCFG_CLKOUT_EN_BIT            0

#define MPU9250_INTMODE_ACTIVEHIGH          0x00
#define MPU9250_INTMODE_ACTIVELOW           0x01

#define MPU9250_INTDRV_PUSHPULL             0x00
#define MPU9250_INTDRV_OPENDRAIN            0x01

#define MPU9250_INTLATCH_50USPULSE          0x00
#define MPU9250_INTLATCH_WAITCLEAR          0x01

#define MPU9250_INTCLEAR_STATUSREAD         0x00
#define MPU9250_INTCLEAR_ANYREAD            0x01

#define MPU9250_INTERRUPT_FF_BIT                7
#define MPU9250_INTERRUPT_MOT_BIT               6
#define MPU9250_INTERRUPT_ZMOT_BIT              5
#define MPU9250_INTERRUPT_FIFO_OFLOW_BIT        4
#define MPU9250_INTERRUPT_I2C_MST_INT_BIT       3
#define MPU9250_INTERRUPT_PLL_RDY_INT_BIT       2
#define MPU9250_INTERRUPT_DMP_INT_BIT           1
#define MPU9250_INTERRUPT_DATA_RDY_BIT          0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU9250_DMPINT_5_BIT                    5
#define MPU9250_DMPINT_4_BIT                    4
#define MPU9250_DMPINT_3_BIT                    3
#define MPU9250_DMPINT_2_BIT                    2
#define MPU9250_DMPINT_1_BIT                    1
#define MPU9250_DMPINT_0_BIT                    0

#define MPU9250_MOTION_MOT_XNEG_BIT             7
#define MPU9250_MOTION_MOT_XPOS_BIT             6
#define MPU9250_MOTION_MOT_YNEG_BIT             5
#define MPU9250_MOTION_MOT_YPOS_BIT             4
#define MPU9250_MOTION_MOT_ZNEG_BIT             3
#define MPU9250_MOTION_MOT_ZPOS_BIT             2
#define MPU9250_MOTION_MOT_ZRMOT_BIT            0

#define MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU9250_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU9250_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU9250_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU9250_PATHRESET_GYRO_RESET_BIT        2
#define MPU9250_PATHRESET_ACCEL_RESET_BIT       1
#define MPU9250_PATHRESET_TEMP_RESET_BIT        0

#define MPU9250_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU9250_DETECT_FF_COUNT_BIT             3
#define MPU9250_DETECT_FF_COUNT_LENGTH          2
#define MPU9250_DETECT_MOT_COUNT_BIT            1
#define MPU9250_DETECT_MOT_COUNT_LENGTH         2

#define MPU9250_DETECT_DECREMENT_RESET        0x0
#define MPU9250_DETECT_DECREMENT_1            0x1
#define MPU9250_DETECT_DECREMENT_2            0x2
#define MPU9250_DETECT_DECREMENT_4            0x3

#define MPU9250_USERCTRL_DMP_EN_BIT             7
#define MPU9250_USERCTRL_FIFO_EN_BIT            6
#define MPU9250_USERCTRL_I2C_MST_EN_BIT         5
#define MPU9250_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU9250_USERCTRL_DMP_RESET_BIT          3
#define MPU9250_USERCTRL_FIFO_RESET_BIT         2
#define MPU9250_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU9250_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU9250_PWR1_DEVICE_RESET_BIT           7
#define MPU9250_PWR1_SLEEP_BIT                  6
#define MPU9250_PWR1_CYCLE_BIT                  5
#define MPU9250_PWR1_GYRO_STANDBY_BIT           4
#define MPU9250_PWR1_TEMP_DIS_BIT               3
#define MPU9250_PWR1_CLKSEL_BIT                 2
#define MPU9250_PWR1_CLKSEL_LENGTH              3

#define MPU9250_CLOCK_INTERNAL              0x00
#define MPU9250_CLOCK_PLL_XGYRO             0x01
#define MPU9250_CLOCK_PLL_YGYRO             0x02
#define MPU9250_CLOCK_PLL_ZGYRO             0x03
#define MPU9250_CLOCK_PLL_EXT32K            0x04
#define MPU9250_CLOCK_PLL_EXT19M            0x05
#define MPU9250_CLOCK_KEEP_RESET            0x07

#define MPU9250_PWR2_LP_WAKE_CTRL_BIT           7
#define MPU9250_PWR2_LP_WAKE_CTRL_LENGTH        2
#define MPU9250_PWR2_STBY_XA_BIT                5
#define MPU9250_PWR2_STBY_YA_BIT                4
#define MPU9250_PWR2_STBY_ZA_BIT                3
#define MPU9250_PWR2_STBY_XG_BIT                2
#define MPU9250_PWR2_STBY_YG_BIT                1
#define MPU9250_PWR2_STBY_ZG_BIT                0

#define MPU9250_WAKE_FREQ_1P25                0x0
#define MPU9250_WAKE_FREQ_2P5                 0x1
#define MPU9250_WAKE_FREQ_5                   0x2
#define MPU9250_WAKE_FREQ_10                  0x3

#define MPU9250_BANKSEL_PRFTCH_EN_BIT           6
#define MPU9250_BANKSEL_CFG_USER_BANK_BIT       5
#define MPU9250_BANKSEL_MEM_SEL_BIT             4
#define MPU9250_BANKSEL_MEM_SEL_LENGTH          5

#define MPU9250_WHO_AM_I_BIT                    6
#define MPU9250_WHO_AM_I_LENGTH                 6

#define MPU9250_DMP_MEMORY_BANKS                8
#define MPU9250_DMP_MEMORY_BANK_SIZE          256
#define MPU9250_DMP_MEMORY_CHUNK_SIZE          16

/* AK8963 Gyro Register Addresses */
#define AK8963_DEFAULT_ADDRESS              0x0C

#define AK8963_WAI                          0x00    /* read-only */

#define AK8963_WAI_DATA                   0x48

#define AK8963_INFO                         0x01
#define AK8963_STI                          0x02
#define AK8963_HXL                          0x03
#define AK8963_HXH                          0xO4
#define AK8963_HYL                          0x05
#define AK8963_HYH                          0x06
#define AK8963_HZL                          0x07
#define AK8963_HZH                          0x08
#define AK8963_ST2                          0x09
#define AK8963_CNTL1                        0x0A    /* read/write */

#define AK8963_CNTL1_PWR_DOWN             0x00      /* power down mode */
#define AK8963_CNTL1_SINGLE_MSRMT         0x01      /* single measurement mode */
#define AK8963_CNTL1_CONT_MSRMT1          0x02      /* continuous measurement 1 mode */
#define AK8963_CNTL1_CONT_MSRMT2          0x06      /* continuous measurement 2 mode */
#define AK8963_CNTL1_EXT_MSRMT            0x04      /* external trigger measurement */
#define AK8963_CNTL1_SELF_TEST            0x08      /* self-test mode */
#define AK8963_CNTL1_FUSE_ROM             0x0F      /* Fuse ROM mode */
#define AK8963_CNTL1_OUT_BITS             0x10      /* output bit, 0 14-bit, 1 16-bit */

#define AK8963_CNTL1_START_BIT               4
#define AK8963_CNTL1_LENGTH                  5

#define AK8963_CNTL2                        0x0B
#define AK8963_ASTC                         0x0C
#define AK8963_TS1                          0x0D
#define AK8963_TS2                          0x0E
#define AK8963_I2CDIS                       0x0F
#define AK8963_ASAX                         0x10    /* read-only */
#define AK8963_ASAY                         0x11
#define AK8963_ASAZ                         0x12
#define AK8963_RSV                          0x13

/* initialize MPU9250, with gyro FS_SEL 250 deg/s and accel FS_SEL 2 g */
void mpu9250_initialize_default (void);
/* initialize MSP9250 providing gyro and accel FS_SEL values */
void mpu9250_initialize (int gyro_fs_sel, int accel_fs_sel);
/* initialize AK8963_magnetometer chip */
void ak8963_initialize (void);

/* check I2C connection with MPU */
bool mpu9250_checkI2C (void);
/* check I2C connection with AK8963 */
bool ak8963_checkI2C (void);

/* get linear and angular accelerations */
void mpu9250_get_accel (float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz);

/* get linear and angular accelerations with temperature */
void mpu9250_get_accel_w_temp (float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *tempc);

/* get linear acceleration only */
void mpu9250_aet_lin_accel (float *ax, float *ay, float *az);
/* get temperature */
void mpu9250_get_tempc (float *tempc);
/* get angular accelerations only */
void mpu9250_get_ang_accel (float *gx, float *gy, float *gz);

/* get magnetic orientation */
void ak8963_get_orientation (float *mx, float *my, float *mz);


#endif /* MPU9250_H_ */

