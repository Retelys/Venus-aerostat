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
	HAL_StatusTypeDef rez = HAL_I2C_Mem_Write(BMP390_I2C, (uint8_t)intf_ptr << 1, reg_addr, 1, reg_data, len, 1000);
	return rez;
}

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
	HAL_StatusTypeDef rez = HAL_I2C_Mem_Read(BMP390_I2C, (uint8_t)intf_ptr << 1, reg_addr, 1, reg_data, len, 1000);
	return rez;
}

static void delay_usec(uint32_t us, void *intf_ptr) {
	uint32_t del = us/1000;
	if (del == 0) {
		del = 2;
	}
	HAL_Delay(del);
}

static int8_t validate_trimming_param(struct bmp3_dev *dev);

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

	BMP390_sens.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_8X;
	BMP390_sens.settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
	BMP390_sens.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
	BMP390_sens.settings.odr_filter.odr = BMP3_ODR_1_5_HZ;
	BMP390_sens.settings.op_mode = BMP3_MODE_NORMAL;
	BMP390_sens.settings.temp_en = BMP3_ENABLE;
	BMP390_sens.settings.press_en = BMP3_ENABLE;

	uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_TEMP_OS | BMP3_SEL_PRESS_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;
	rslt = bmp3_set_sensor_settings(settings_sel, &BMP390_sens);
	if (rslt != BMP3_OK) {
		return 3;
	}

	rslt = bmp3_set_op_mode(&BMP390_sens);
	if (rslt != BMP3_OK) {
		return 4;
	}

	rslt = validate_trimming_param(&BMP390_sens);
	if (rslt != BMP3_OK) {
		return 5;
	}
}

int8_t BMP390_read(struct bmp3_data* output) {
	return bmp3_get_sensor_data(BMP3_TEMP | BMP3_PRESS, output, &BMP390_sens);
}

static int8_t cal_crc(uint8_t seed, uint8_t data);

static int8_t validate_trimming_param(struct bmp3_dev *dev) {
  int8_t rslt;
  uint8_t crc = 0xFF;
  uint8_t stored_crc;
  uint8_t trim_param[21];
  uint8_t i;

  rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
  if (rslt == BMP3_OK) {
    for (i = 0; i < 21; i++) {
      crc = (uint8_t)cal_crc(crc, trim_param[i]);
    }

    crc = (crc ^ 0xFF);
    rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
    if (stored_crc != crc) {
      rslt = -1;
    }
  }

  return rslt;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data) {
  int8_t poly = 0x1D;
  int8_t var2;
  uint8_t i;

  for (i = 0; i < 8; i++) {
    if ((seed & 0x80) ^ (data & 0x80)) {
      var2 = 1;
    } else {
      var2 = 0;
    }

    seed = (seed & 0x7F) << 1;
    data = (data & 0x7F) << 1;
    seed = seed ^ (uint8_t)(poly * var2);
  }

  return (int8_t)seed;
}
