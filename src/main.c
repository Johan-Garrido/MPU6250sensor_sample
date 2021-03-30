/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#define MPU_addr 0x68 //I2C address of the MPU-6050
const struct device *mpu6050;

//I2C structure
typedef struct  i2cData {
  float AcX;
  float AcY;
  float AcZ;
  float GyX;
  float GyY;
  float GyZ;
  float Temp;
  }i2cData;


void input_read(i2cData *sensorData){
  int8_t data[14]; // Buffer to transmit data in I2C protocol
  data[0] = 0x3B; // Starting with register 0x3B (ACCEL_XOUT_H)
 
  i2c_write(mpu6050, data, 1, MPU_addr);  
  i2c_read(mpu6050, data, 14, MPU_addr);

  sensorData->AcX = ((double)data[0])+((double)(data[1]) / 1000000); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  sensorData->AcY = ((double)data[2])+((double)(data[3]) / 1000000); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT
  sensorData->AcZ = ((double)data[4])+((double)(data[5]) / 1000000); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  sensorData->Temp = (((((double)data[6])+((double)(data[7]) / 1000000)) + 521 ) + (35 * 340)) / 340; // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  sensorData->GyX = ((double)data[8])+((double)(data[9]) / 1000000); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  sensorData->GyY = ((double)data[10])+((double)(data[11]) / 1000000); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  sensorData->GyZ = ((double)data[12])+((double)(data[13]) / 1000000); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
//Function created to initialize the communitaction I2C between the board and MPU-6050 
void init_mpu(){
  uint8_t data[2];
  data[0] = 0x6B; // PWR_MGMT_1 register
  data[1] = 0x00; // Wakes up the MPU-6050
  i2c_write(mpu6050, data, 2, MPU_addr);
}

void main(void)
{
  struct i2cData sensorData;  
  mpu6050 = device_get_binding("I2C_0");

  if (!mpu6050) {
    printf("Failed to find sensor I2C\n");
    return;
  }
  else
    init_mpu();
  
  while(1){
    input_read(&sensorData);
    printf("Data: %f Cel\n"
      "  accel % f % f % f m/s/s\n"
      "  gyro  % f % f % f rad/s\n",
      sensorData.Temp,
      sensorData.AcX,
      sensorData.AcY,
      sensorData.AcZ,
      sensorData.GyX,
      sensorData.GyY,
      sensorData.GyZ);
     k_sleep(K_SECONDS(2));
  }
}
