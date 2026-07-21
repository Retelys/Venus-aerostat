#include <stdio.h>
#include <string.h>

#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

void SD_Write(void* data, uint8_t len, char* file);
