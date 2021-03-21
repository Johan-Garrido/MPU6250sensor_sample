/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>

typedef struct {
  uint16_t AcX;
  uint16_t AcY;
  uint16_t AcZ;
  uint16_t GyX;
  uint16_t GyY;
  uint16_t GyZ;
  uint16_t Temp;
} i2cData;

int input_read(i2cData *i2cData){
  return 1;
}

void main(void)
{
  const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
	const struct device *mpu6050 = device_get_binding(label);

	if (!mpu6050) {
		printf("Failed to find sensor %s\n", label);
		return;
	}
        struct i2cData sensorData;
  while(1){
  input_read(sensorData);
  printf("Data: %g Cel\n"
		       "  accel % i % i % i m/s/s\n"
		       "  gyro  % i % i % i rad/s\n",
		       sensorData->Temp,
		       sensorData->AcX,
		       sensorData->AcY,
		       sensorData->AcZ,
		       sensorData->GyX,
		       sensorData->GyY,
		       sensorData->GyZ);
  k_sleep(K_SECONDS(2));
  }
}
