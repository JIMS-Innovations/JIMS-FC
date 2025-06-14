/**
 * @file IMU.h
 * @author Jesutofunmi Kupoluyi (innovationsjims@gmail.com)
 * @brief This is the register map/definition for the MPU6050 (IMU)
 * @version 0.1
 * @date 2022-08-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

            /* ADDRESS DEFINITIONS*/

#define ADDRESS       0X68 // IMU I2C address

#define SELF_TEST_X   0X0D // R/W
#define SELF_TEST_Y   0X0E // R/w
#define SELF_TEST_Z   0X0F // R/W
#define SELF_TEST_A   0X10 // R/W
#define SMPLRT_DIV    0X19 // R/W
#define CONFIG        0X1A // R/W
#define GYRO_CONFIG   0X1B // R/W
#define ACCEL_CONFIG  0X1C // R/W
#define FIFO_EN       0X23 // R/W
#define INT_PIN_CFG   0X37 // R/W
#define INT_ENABLE    0X38 // R/W
#define INT_STATUS    0X3A // R/W
#define ACCEL_XOUT_H  0X3B // R
#define ACCEL_XOUT_L  0X3C // R
#define ACCEL_YOUT_H  0X3D // R
#define ACCEL_YOUT_L  0X3E // R
#define ACCEL_ZOUT_H  0X3F // R
#define ACCEL_ZOUT_L  0X40 // R
#define TEMP_OUT_H    0X41 // R
#define TEMP_OUT_L    0X42 // R
#define GYRO_XOUT_H   0X43 // R
#define GYRO_XOUT_L   0X44 // R
#define GYRO_YOUT_H   0X45 // R
#define GYRO_YOUT_L   0X46 // R
#define GYRO_ZOUT_H   0X47 // R
#define GYRO_ZOUT_L   0X48 // R
#define USER_CTRL     0X6A // R/W
#define PWR_MGMT_1    0X6B // R/W
#define PWR_MGMT_2    0X6C // R/W

            /*  CONSTANTS   */

#define RAW_TO_MS2 4096
#define RAW_TO_DEGS 65.5
//#define RAD_TO_DEG 57.2957795131f
#define GRAVITY 9.8100000000f

            /* DATA ABSTRACTIONS */
            

            /* FUNCTION PROTOTYPES */

void IMU_init();
void IMU_data( int *acc_x, int *acc_y, int *acc_z, int *gyro_x, int *gyro_y, int *gyro_z, int *temp);