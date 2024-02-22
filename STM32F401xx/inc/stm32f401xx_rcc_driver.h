/*
 * stm32f401xx_rcc_driver.h
 *
 *  Created on: Jan 18, 2024
 *      Author: Ahmed Samir
 */

#ifndef INC_STM32F401XX_RCC_DRIVER_H_
#define INC_STM32F401XX_RCC_DRIVER_H_

#include"stm32f401xx.h"
uint32_t RCC_GetPllOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F401XX_RCC_DRIVER_H_ */
