/*
 * led.c
 *
 *  Created on: Nov 21, 2025
 *      Author: maram
 */
#include"led.h"
#include <stdint.h>
#include "main.h"
extern SPI_HandleTypeDef hspi3;

void write_MCP23017(uint8_t registre, uint8_t value)
{
	uint8_t commande[3];
	commande[0]=0x40; //
	commande[1]=registre;
	commande[2]=value;

	HAL_GPIO_WritePin(GPIOA, VU_nRESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, VU_nCS_Pin, GPIO_PIN_RESET); //SC=0 pour l'envoie
	HAL_SPI_Transmit(&hspi3, commande, 3, HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB,VU_nCS_Pin, GPIO_PIN_SET);



}

uint8_t read_MCP23S17(uint8_t reg) {
    uint8_t data[3];
    uint8_t rxData[1];

    data[0] = 0x41;
    data[1] = reg;
    data[2] = 0x00;

    HAL_GPIO_WritePin(GPIOB, VU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, data, data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, VU_nCS_Pin, GPIO_PIN_SET);

    return data[2];
}

void init_MCP23017(void)
{
write_MCP23017(0x00,0x00); // les oins A en sortie
write_MCP23017(0x01,0x00); // les pins B en sortie
write_MCP23017(0x12,0xFF);
write_MCP23017(0x13,0x0FF);


}
void Test_First_LED(void) {
	write_MCP23017(0x13, 0xFE);
	write_MCP23017(0x12, 0xFE);
}
void Blink_All_LEDs(void) {
	while (1) {
		write_MCP23017(0x13, 0xFF);
		HAL_Delay(500);
		write_MCP23017(0x13, 0x00);
		HAL_Delay(500);
	}
}
