# mpu9250-driverlib
## MPU-9250 Library for TI MSP432 using TI Driverlib

The InvenSense MPU-9250 combines a 3-axis accelerometer, 3-axis gyroscope and 3-asix magnetometer in a postage-stamp sized package. There are a number of libraries around, but few and provide a thorough interface to the MPU-9250. This library provides full suport of the accelerometer, gyroscope and magnetometer including retriving and use of the Axis Sensitify Adjustments for the magnetometer encoded in the FUSE_ROM of the AK8963 chip.

This library draws from [guilhermelionzo /
MPU9250_driverlib](https://github.com/guilhermelionzo/MPU9250_driverlib) and extends the functionality providing magnetometer and temperature sensor access

### The Library

The library is provided in `mpu9250.h` and `mpu9250.c`. The library uses the TI-Driverlib interface. The header contains defines for the complete Register_Map for the MPU9250 and AK8063 chips along with individual bit-level defines for most registers that will need to be accessed or set. The library and example build fine with either CCS or gcc-arm-none-eabi.

### The Example Program

An example program is provided in `example_mpu9250.c` that reads all sensor values from the MPU-9250 and outputs them to a serial terminal using UART. The support source files and headers for the example program are contained in the `src` and `include` directories. 

If using CCS simply create an empty driverlib project. "Add" all source files to the project and copy the headers to a directory so CCS can find them and compile. (you will need to add the location of the headers in *"Show Build Settings -> Build -> Compiler -> Directores"* Then click Add and browse to the location of the headers to include that as part of the include search path. No other changes are needed and the code is C89 compatible so CCS defaults are fine.

The comm-parameters for the serial terminal used with the example are 115K-N-8-1 with the config assuming the default 3MHz SMCLK. (adjust as needed, See [TI BAud Rate Calculator](http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html))

### Hardware Setup for Example

Since the MPU-9250 uses I2C to communicate with the sensors, two wires are all that are needed the example uses P1.6 for SDA and P1.7 for SCL. Simply connect Pin 1.6 on the MSP to the SDA pin on the MPU-9250 and Pin 1.7 to SCL. **NOTE:** because the magnetometer is on the 3rd-party AK8963 chip that uses the extended I2C bus on the MPU-9250, you must connect the SDA and EDA pins together and the SCL and ECL pins together to read the magnetometer values, e.g.

            MPU9250
         +-----------+
         | VCC       |
         | GND       |
         | SCL----+  |
         | SDA-+  |  |
         | EDA-+  |  |
         | ECL----+  |
         | ...       |

### Example Use

When you connect the MSP to a serial terminal and load the example, the program will output validation of chip initializations and I2C communication checks for both chips. It will then output all sensor data and wait for further instructions, e.g.

        MPU9250: initialized
        AK8963 : initialized
        
        MPU9250 I2C: connection established.
        AK8963 I2C : connection established.
        
        Usage:
          switch1 (P1.1) - starts data update
          switch2 (P1.4) - stops data update
        
        Accelerations (linear/angular), Magnetic Field, and Temperature
        
          accel: {   -0.22,     0.01,     0.82  }
          gyro : {   -1.66,     0.24,    -4.01  }
          mag  : {  -18.85,    43.11,    69.11  }
          temp : {   23.30 C /    73.93 F  }


To continually sample and output the sensor values once per second, press Switch1 (P1.1) and the sersor values will be continually output:

        Automatic data update started:
        
          accel: {   -0.23,     0.02,     0.81  }
          gyro : {    1.89,    -1.47,    -3.97  }
          mag  : {  -19.36,    42.94,    68.94  }
          temp : {   23.31 C /    73.95 F  }
        
          accel: {   -0.21,     0.01,     0.82  }
          gyro : {   -1.69,     0.18,    -4.01  }
          mag  : {  -18.51,    42.05,    68.77  }
          temp : {   23.32 C /    73.98 F  }
          
          ...

Pressing Switch2 (P1.4) suspends continual output.

### Future Plans

There are a few other TODOs, but the mpu9250 library functionality is substantially complete. A clean up of the use I2C pin selection will be made easier by adding a macro or two to take the port and pin number set by the user and stringify into the MSP for (e.g. `P1.6`, and so on). If you find any issues, let me know. (also, be sure it is a software issues and not a faky MPU-9250 chip -- there have been a lot lately, dead gyros, temp sensors 8 deg. apart, etc.) I'm batting about 30% on fully functional chips. The first had a dead gyro, next had dead z-gyro (value stuck between 3.96 and 4.1, the third is fine.


