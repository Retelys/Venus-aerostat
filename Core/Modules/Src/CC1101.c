//# Address Config = No address check
//# Base Frequency = 433.099792
//# CRC Autoflush = false
//# CRC Enable = false
//# Carrier Frequency = 433.099792
//# Channel Number = 0
//# Channel Spacing = 49.987793
//# Data Format = Normal mode
//# Data Rate = 1.00112
//# Deviation = 12.695313
//# Device Address = 0
//# Manchester Enable = false
//# Modulated = true
//# Modulation Format = 2-FSK
//# PA Ramping = false
//# Packet Length = 255
//# Packet Length Mode = Variable packet length mode.
//# Preamble Count = 2
//# RX Filter BW = 203.125000
//# Sync Word Qualifier Mode = 30/32 sync word bits detected
//# TX Power = 0
//# Whitening = false
//
// Rf settings for CC1100
//

#include "CC1101.h"

#define PA_TABLE {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
const uint8_t pa_table[] = PA_TABLE;

void CC1101_write_reg(uint8_t addr, uint8_t data) {
	uint8_t bytes[] = {addr , data};
	uint8_t rx[2];
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);
	HAL_SPI_TransmitReceive(CC1101_SPI_Ch, bytes, rx, sizeof(bytes), 1000);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_SPI_SCK, GPIO_PIN_RESET);
}

void CC1101_init() {
	uint8_t sres[] = {0x30};
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_SPI_SCK, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_SPI_MOSI, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);
	HAL_SPI_Transmit(CC1101_SPI_Ch, sres, sizeof(sres), 1000);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);

	CC1101_conf();
}

void CC1101_conf() {
	CC1101_write_reg(0x0000, 0x29); //IOCFG2 GDO2 Output Pin Configuration
	CC1101_write_reg(0x0001, 0x2E); //IOCFG1 GDO1 Output Pin Configuration
	CC1101_write_reg(0x0002, 0x06); //IOCFG0 GDO0 Output Pin Configuration
	CC1101_write_reg(0x0003, 0x07); //FIFOTHR RX FIFO and TX FIFO Thresholds
	CC1101_write_reg(0x0004, 0x1D); //SYNC1 Sync Word, High Byte
	CC1101_write_reg(0x0005, 0xDA); //SYNC0 Sync Word, Low Byte
	CC1101_write_reg(0x0006, 0xFF); //PKTLEN Packet Length
	CC1101_write_reg(0x0007, 0x04); //PKTCTRL1 Packet Automation Control
	CC1101_write_reg(0x0008, 0x01); //PKTCTRL0 Packet Automation Control
	CC1101_write_reg(0x0009, 0x00); //ADDR Device Address
	CC1101_write_reg(0x000A, 0x00); //CHANNR Channel Number
	CC1101_write_reg(0x000B, 0x0F); //FSCTRL1 Frequency Synthesizer Control
	CC1101_write_reg(0x000C, 0x00); //FSCTRL0 Frequency Synthesizer Control
	CC1101_write_reg(0x000D, 0x10); //FREQ2 Frequency Control Word, High Byte
	CC1101_write_reg(0x000E, 0xA8); //FREQ1 Frequency Control Word, Middle Byte
	CC1101_write_reg(0x000F, 0x5E); //FREQ0 Frequency Control Word, Low Byte
	CC1101_write_reg(0x0010, 0x82); //MDMCFG4 Modem Configuration
	CC1101_write_reg(0x0011, 0x02); //MDMCFG3 Modem Configuration
	CC1101_write_reg(0x0012, 0x02); //MDMCFG2 Modem Configuration
	CC1101_write_reg(0x0013, 0x02); //MDMCFG1 Modem Configuration
	CC1101_write_reg(0x0014, 0xF8); //MDMCFG0 Modem Configuration
	CC1101_write_reg(0x0015, 0x14); //DEVIATN Modem Deviation Setting
	CC1101_write_reg(0x0016, 0x07); //MCSM2 Main Radio Control State Machine Configuration
	CC1101_write_reg(0x0017, 0x30); //MCSM1 Main Radio Control State Machine Configuration
	CC1101_write_reg(0x0018, 0x14); //MCSM0 Main Radio Control State Machine Configuration
	CC1101_write_reg(0x0019, 0x36); //FOCCFG Frequency Offset Compensation Configuration
	CC1101_write_reg(0x001A, 0x6C); //BSCFG Bit Synchronization Configuration
	CC1101_write_reg(0x001B, 0x03); //AGCCTRL2 AGC Control
	CC1101_write_reg(0x001C, 0x40); //AGCCTRL1 AGC Control
	CC1101_write_reg(0x001D, 0x91); //AGCCTRL0 AGC Control
	CC1101_write_reg(0x001E, 0x87); //WOREVT1 High Byte Event0 Timeout
	CC1101_write_reg(0x001F, 0x6B); //WOREVT0 Low Byte Event0 Timeout
	CC1101_write_reg(0x0020, 0xF8); //WORCTRL Wake On Radio Control
	CC1101_write_reg(0x0021, 0x56); //FREND1 Front End RX Configuration
	CC1101_write_reg(0x0022, 0x10); //FREND0 Front End TX Configuration
	CC1101_write_reg(0x0023, 0xE9); //FSCAL3 Frequency Synthesizer Calibration
	CC1101_write_reg(0x0024, 0x2A); //FSCAL2 Frequency Synthesizer Calibration
	CC1101_write_reg(0x0025, 0x00); //FSCAL1 Frequency Synthesizer Calibration
	CC1101_write_reg(0x0026, 0x1F); //FSCAL0 Frequency Synthesizer Calibration
	CC1101_write_reg(0x0027, 0x41); //RCCTRL1 RC Oscillator Configuration
	CC1101_write_reg(0x0028, 0x00); //RCCTRL0 RC Oscillator Configuration
	CC1101_write_reg(0x0029, 0x59); //FSTEST Frequency Synthesizer Calibration Control
	CC1101_write_reg(0x002A, 0x7F); //PTEST Production Test
	CC1101_write_reg(0x002B, 0x3F); //AGCTEST AGC Test
	CC1101_write_reg(0x002C, 0x88); //TEST2 Various Test Settings
	CC1101_write_reg(0x002D, 0x31); //TEST1 Various Test Settings
	CC1101_write_reg(0x002E, 0x09); //TEST0 Various Test Settings

	uint8_t to_send = 0x3E | (1 << 6);
	HAL_SPI_Transmit(CC1101_SPI_Ch, &to_send, 1, 1000);
	HAL_SPI_Transmit(CC1101_SPI_Ch, pa_table, sizeof(pa_table), 1000);
}

void CC1101_TX(uint8_t *data, uint8_t len_data) {
	uint8_t buf[65];
	buf[0] = 0x3F | (1 << 6);
	buf[1] = len_data;
	for (uint8_t i = 0; i < len_data; i++) {
		buf[i + 2] = data[i];
	}

	uint8_t cmd = 0x3B;
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);
	HAL_SPI_Transmit(CC1101_SPI_Ch, &cmd, sizeof(cmd), 1000);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);
	HAL_SPI_Transmit(CC1101_SPI_Ch, buf, sizeof(buf), 1000);
	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(CC1101_GPIO_PORT, CC1101_SPI_MISO) == GPIO_PIN_SET);

	cmd = 0x35;
	HAL_SPI_Transmit(CC1101_SPI_Ch, &cmd, sizeof(cmd), 1000);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CC1101_GPIO_PORT, CC1101_SPI_SCK, GPIO_PIN_RESET);
//	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != GPIO_PIN_SET);
//	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != GPIO_PIN_RESET);
}
