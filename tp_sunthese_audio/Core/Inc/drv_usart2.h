/*
 * drv_usart2.h
 *
 *  Created on: Nov 20, 2025
 *      Author: maram
 */

#ifndef INC_DRV_USART2_H_
#define INC_DRV_USART2_H_

#include <stdint.h>

uint8_t drv_uart2_receive(char * pData, uint16_t size);
uint8_t drv_uart2_transmit(const char * pData, uint16_t size);


#endif /* INC_DRV_USART2_H_ */
