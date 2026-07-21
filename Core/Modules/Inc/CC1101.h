#include "stm32f1xx_hal.h"
#include "main.h"
#include "spi.h"

#define CC1101_GPIO_PORT		GPIOA
#define CC1101_CS_Pin			GPIO_PIN_4
#define CC1101_SPI_Ch			&hspi1
#define CC1101_SPI_MOSI			GPIO_PIN_7
#define CC1101_SPI_MISO			GPIO_PIN_6
#define CC1101_SPI_SCK			GPIO_PIN_5

void CC1101_write_reg(uint8_t addr, uint8_t data);
void CC1101_conf(void);
void CC1101_init(void);
void CC1101_TX(uint8_t *data, uint8_t len_data);
