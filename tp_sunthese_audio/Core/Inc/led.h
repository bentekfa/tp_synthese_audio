/*
 * led.h
 *
 *  Created on: Nov 21, 2025
 *      Author: maram
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdint.h>

typedef struct {
    void (*init)(void);
    void (*write)(uint8_t reg, uint8_t value);
    void (*test_first_led)(void);
    void (*chenillard)(void);
    void (*blink_all)(void);
    uint8_t (*read)(uint8_t reg);
} LED_Driver_t;


void write_MCP23017(uint8_t reg, uint8_t value);
void init_MCP23017(void);
void Test_First_LED(void);
void Blink_All_LEDs(void);
void LED_Chenillard(void);
void LED_Driver_Init(LED_Driver_t *driver);


#endif /* INC_LED_H_ */
