#pragma once

#include "stm32f1xx_hal.h"

typedef struct __attribute__((packed)){
	float x;
	float y;
	float z;
} XYZ;

#define MPU6050_I2C		hi2c1

void MPU6050_init(void); //Initialize the MPU
void MPU6050_Read_Accel (XYZ* output); //Read MPU Accelerator
void MPU6050_Read_Gyro (XYZ* output); //Read MPU Gyroscope
