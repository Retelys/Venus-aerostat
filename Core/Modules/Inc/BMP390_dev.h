#pragma once
#include "BMP390_defs.h"

#define BMP390_I2C		&hi2c1

extern struct bmp3_dev BMP390_sens;
uint8_t BMP390_init();
int8_t BMP390_read(struct bmp3_data* output);
