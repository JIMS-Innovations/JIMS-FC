/**
 * @file main.cpp
 * @author Jesutofunmi Kupoluyi (innovationsjims@gmail.com)
 * @brief This is a flight controller software
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */
// Including required libraries
#include <Arduino.h>
#include <Wire.h>
//#include <EEPROM.h>
#include "IMU.h"

#define LOOP_TIME 4  // 4 microseconds
#define CALIB_NO 2000

// Variables
int acc[3], gyro[3], gyro_cal[3], temp;                                                 //raw acceelerometer and gyro values
float tempC, acc_ms2[3], gyro_degs[3], phi[2], theta[2];                                                       //converted accelerometer and gyro values
char out[100], str_acc_x_ms2[10], str_acc_y_ms2[10], str_acc_z_ms2[10], str_gyro_x_degs[10], str_gyro_y_degs[10], str_gyro_z_degs[10], str_tempC[10];  //string buffers for the accelerometer and gyro values
unsigned long previousMillis;
unsigned long currentMillis;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  IMU_init();
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("IMU Initialized!");

  Serial.print("Calibrating, Do not move!!!");
  for (int i = 0; i < CALIB_NO; i++) {
    if (i % 125 == 0) Serial.print(".");
    IMU_data(&acc_x, &acc_y, &acc_z, &gyro[0], &gyro[1], &gyro[2], &temp);
    gyro_cal[0] += gyro[0];
    gyro_cal[1] += gyro[1];
    gyro_cal[2] += gyro[2];
    delay(3);
  }
  gyro_cal[0] /= 2000;
  gyro_cal[1] /= 2000;
  gyro_cal[2] /= 2000;

  previousMillis = 0;
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= LOOP_TIME) {
    IMU_data(&acc[0], &acc[1], &acc[3], &gyro[0], &gyro[1], &gyro[2], &temp);

    acc_x_ms2 = acc_x / RAW_TO_MS2;
    acc_y_ms2 = acc_y / RAW_TO_MS2;
    acc_z_ms2 = acc_z / RAW_TO_MS2;

    // gyro_x_degs += gyro[0] / 250 / RAW_TO_DEGS;
    // gyro_y_degs += gyro[1] / 250 / RAW_TO_DEGS;
    // gyro_z_degs += gyro[2] / 250 / RAW_TO_DEGS;

    gyro[0] -= gyro_cal[0];
    gyro[1] -= gyro_cal[1];
    gyro[2] -= gyro_cal[2];

    gyro_x_degs += gyro[0] * 0.0000611;
    gyro_y_degs += gyro[1] * 0.0000611;
    gyro_z_degs += gyro[2] * 0.0000611;
    tempC = (temp / 340) + 36.5;

    dtostrf(acc_ms2[0], 4, 2, str_acc_x_ms2);
    dtostrf(acc_ms2[1], 4, 2, str_acc_y_ms2);
    dtostrf(acc_ms2[2], 4, 2, str_acc_z_ms2);
    dtostrf(gyro_degs[0], 4, 2, str_gyro_x_degs);
    dtostrf(gyro_degs[0], 4, 2, str_gyro_y_degs);
    dtostrf(gyro_degs[0], 4, 2, str_gyro_z_degs);
    dtostrf(tempC, 4, 2, str_tempC);


    // sprintf(out, "a_x: %s, a_y: %s, a_z: %s, g_x: %s, g_y: %s, g_z: %s, temp: %s", str_acc_x_ms2, str_acc_y_ms2, str_acc_z_ms2, str_gyro_x_degs, str_gyro_y_degs, str_gyro_z_degs, str_tempC);
    sprintf(out, "%s, %s, %s, %s, %s, %s, %s", str_acc_x_ms2, str_acc_y_ms2, str_acc_z_ms2, str_gyro_x_degs, str_gyro_y_degs, str_gyro_z_degs, str_tempC);
    // phi = atan(acc_y/acc_z) * RAD_TO_DEG;
    //  theta = asin(acc_x_ms2/GRAVITY) * RAD_TO_DEG;
    Serial.println(out);
    // Serial.println(tempC);
    previousMillis = currentMillis;
  }
  // Serial.println(startMillis);
  // Serial.println(currentMillis);
}
