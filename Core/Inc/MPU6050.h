#include "stm32f1xx_hal.h"

typedef struct {
	float x;
	float y;
	float z;
} XYZ;

void MPU6050_init(void); //Initialize the MPU
void MPU6050_Read_Accel (XYZ* output); //Read MPU Accelerator
void MPU6050_Read_Gyro (XYZ* output); //Read MPU Gyroscope
