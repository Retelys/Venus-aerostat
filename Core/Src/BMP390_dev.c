#include "BMP390_dev.h"
#include "BMP390.h"
#include "i2c.h"

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);

struct bmp3_dev BMP390_sens = {
	.chip_id = 0x76,
	.intf = BMP3_I2C_INTF,
	.read = &i2c_read,
	.write = &i2c_write,
	.delay_us = &delay_usec,
	.intf_ptr = (void*)0x76,
	.dummy_byte = 0
};

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
        void *intf_ptr) {
	HAL_StatusTypeDef rez = HAL_I2C_Mem_Write(&hi2c1, (uint8_t)intf_ptr << 1, reg_addr, 1, reg_data, len, 1000);
	return rez;
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
	HAL_StatusTypeDef rez = HAL_I2C_Mem_Read(&hi2c1, (uint8_t)intf_ptr << 1, reg_addr, 1, reg_data, len, 1000);
	return rez;
}

static void delay_usec(uint32_t us, void *intf_ptr) {
	HAL_Delay(us/1000);
}

uint8_t BMP390_init(void) {
	int8_t rslt = BMP3_OK;
	rslt = bmp3_soft_reset(&BMP390_sens);
	if (rslt != BMP3_OK) {
		return 1;
	}
	rslt = bmp3_init(&BMP390_sens);
	if (rslt != BMP3_OK) {
		return 2;
	}

	BMP390_sens.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	BMP390_sens.settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
	BMP390_sens.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_DISABLE;
	BMP390_sens.settings.odr_filter.odr = BMP3_ODR_25_HZ;
	BMP390_sens.settings.op_mode = BMP3_MODE_FORCED;
	BMP390_sens.settings.temp_en = BMP3_ENABLE;

	uint16_t settings_sel = BMP3_SEL_TEMP_EN | BMP3_SEL_TEMP_OS | BMP3_SEL_PRESS_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;
	rslt = bmp3_set_sensor_settings(settings_sel, &BMP390_sens);
	if (rslt != BMP3_OK) {
		return 3;
	}

	rslt = bmp3_set_op_mode(&BMP390_sens);
	if (rslt != BMP3_OK) {
		return 4;
	}
}

int8_t BMP390_read(struct bmp3_data* output) {
	return bmp3_get_sensor_data(3, output, &BMP390_sens);
}
