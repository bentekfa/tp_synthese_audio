/*
 *  sgtl5000.c
 *
 *  Created on: Nov 28, 2025
 *      Author: maram
 */


#include  "sgtl5000.h"
#include "main.h"


HAL_StatusTypeDef sgtl5000_i2c_write_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t value) {
	uint8_t data[4];
	data[0] = (reg >> 8) & 0xFF; // High byte of the register
	data[1] = reg & 0xFF;        // Low byte of the register
	data[2] = (value >> 8) & 0xFF; // High byte of the value
	data[3] = value & 0xFF;        // Low byte of the value

	return HAL_I2C_Master_Transmit(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, data, 4, HAL_MAX_DELAY);
}


HAL_StatusTypeDef sgtl5000_i2c_set_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask) {
	uint16_t current_value;
	HAL_StatusTypeDef ret = sgtl5000_i2c_read_register(h_sgtl5000, reg, &current_value);
	if (ret != HAL_OK) return ret;

	return sgtl5000_i2c_write_register(h_sgtl5000, reg, current_value | mask);
}


HAL_StatusTypeDef sgtl5000_i2c_clear_bit(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t mask) {
	uint16_t current_value;
	HAL_StatusTypeDef ret = sgtl5000_i2c_read_register(h_sgtl5000, reg, &current_value);
	if (ret != HAL_OK) return ret;

	return sgtl5000_i2c_write_register(h_sgtl5000, reg, current_value & ~mask);
}


HAL_StatusTypeDef sgtl5000_i2c_read_register(h_sgtl5000_t *h_sgtl5000, uint16_t reg, uint16_t *value) {
	uint8_t reg_addr[2];
	uint8_t data[2];

	reg_addr[0] = (reg >> 8) & 0xFF; // High byte of the register
	reg_addr[1] = reg & 0xFF;        // Low byte of the register

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, reg_addr, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) return ret;

	ret = HAL_I2C_Master_Receive(h_sgtl5000->hi2c, h_sgtl5000->i2c_address, data, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) return ret;

	*value = (data[0] << 8) | data[1];
	return HAL_OK;
}

HAL_StatusTypeDef sgtl5000_init(h_sgtl5000_t * h_sgtl5000)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint16_t mask;


	mask = 0x6AFF;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ANA_POWER, mask);


	mask = (1 << 5) | (1 << 6);
	sgtl5000_i2c_set_bit(h_sgtl5000, SGTL5000_CHIP_LINREG_CTRL, mask);


	mask = 0x01FF;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_REF_CTRL, mask);


	mask = 0x031E;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_LINE_OUT_CTRL, mask);


	mask = 0x1106;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_SHORT_CTRL, mask);


	mask = 0x0004;
	//	mask = 0x0000;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ANA_CTRL, mask);


	mask = 0x6AFF;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ANA_POWER, mask);

	mask = 0x0073;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_DIG_POWER, mask);


	mask =  0x0505;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_LINE_OUT_VOL, mask);


	mask = 0x0004;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_CLK_CTRL, mask);

	mask = 0x0130;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_I2S_CTRL, mask);

	mask = 0x0000;
	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_ADCDAC_CTRL, mask);

	mask = 0x3C3C;

	sgtl5000_i2c_write_register(h_sgtl5000, SGTL5000_CHIP_DAC_VOL, mask);

	return ret;
}
