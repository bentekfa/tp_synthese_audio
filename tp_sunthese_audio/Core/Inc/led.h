/*
 * led.h
 *
 *  Created on: Nov 21, 2025
 *      Author: maram
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdint.h>
void write_MCP23017(uint8_t reg, uint8_t value);
void init_MCP23017(void);
void Test_First_LED(void);
void Blink_All_LEDs(void);
void LED_Chenillard(void);

#endif /* INC_LED_H_ */
