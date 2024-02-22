/*
 * stm32f401xx_gpio_drivers.h
 *
 *  Created on: Nov 1, 2023
 *      Author: ahmed samir
 */

#ifndef DRIVER_INC_STM32F401XX_GPIO_DRIVERS_H_
#define DRIVER_INC_STM32F401XX_GPIO_DRIVERS_H_

#include "stm32f401xx.h"


/*                    ************************************************
                      *   handle structure for GPIOx                 *
                      ************************************************           */
typedef struct
{
	uint8_t GPIO_PinNumber ;          /* <possible values from @GPIO_PIN_NUMBERS >   */
	uint8_t GPIO_PinMode ;            /* <possible values from @GPIO_PIN_MODES >   */
	uint8_t GPIO_PinSpeed ;
	uint8_t GPIO_PinPuPdControl ;
	uint8_t GPIO_PinOPType ;
	uint8_t GPIO_PinAltFunMode ;
}GPIO_PinConfig_t ;


typedef struct
{
    GPIO_RegDef_t *pGPIOx ;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*                    ************************************************
                      *  GPIO pin number
                      *  @GPIO_PIN_NUMBERS                      *
                      ************************************************           */
#define GPIO_PIN_NO_0                 0
#define GPIO_PIN_NO_1                 1
#define GPIO_PIN_NO_2                 2
#define GPIO_PIN_NO_3                 3
#define GPIO_PIN_NO_4                 4
#define GPIO_PIN_NO_5                 5
#define GPIO_PIN_NO_6                 6
#define GPIO_PIN_NO_7                 7
#define GPIO_PIN_NO_8                 8
#define GPIO_PIN_NO_9                 9
#define GPIO_PIN_NO_10                10
#define GPIO_PIN_NO_11                11
#define GPIO_PIN_NO_12                12
#define GPIO_PIN_NO_13                13
#define GPIO_PIN_NO_14                14
#define GPIO_PIN_NO_15                15

/*                    ************************************************
                      *  GPIO possible modes
                      *  @GPIO_PIN_MODES                      *
                      ************************************************           */
#define GPIO_MODE_IN                 0
#define GPIO_MODE_OUT                1
#define GPIO_MODE_ALTFN              2
#define GPIO_MODE_ANALOG             3
#define GPIO_MODE_IT_FT              4
#define GPIO_MODE_IT_RT              5
#define GPIO_MODE_IT_RFT             6

/*                    ************************************************
                      *  GPIO Pin possible output types               *
                      ************************************************           */
#define GPIO_OP_TYPE_PP              0
#define GPIO_OP_TYPE_OD              1


/*                    ************************************************
                      *  GPIO Pin possible Speed Modes              *
                      ************************************************           */
#define GPIO_OP_SPEED_LOW            0
#define GPIO_OP_SPEED_MEDIUM         1
#define GPIO_OP_SPEED_FAST           2
#define GPIO_OP_SPEED_HIGH           3

/*                    ************************************************
                      *  GPIO Pin pull up and Pull down             *
                      ************************************************           */
#define GPIO_NO_PUPD                 0
#define GPIO_PIN_PU                  1
#define GPIO_PIN_PD                  2


/*                    ************************************************
                      *   APIs supported by this driver               *
                      ************************************************           */

/*****************************
 * peripheral clock setup
*************************** */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnOrDis);

                         /************************************/


/**********************************
 * initiate and De-init the GPIOs *
**********************************/

void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

                        /************************************/

/*************************
 * Data read and write   *
 ************************/

uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);      // read Data from one pin only
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *pGPIOx);                         // read Data from the whole port

void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber  ,uint8_t Value);     // Write Data to one pin only
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value);                       // Write Data to the whole port

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumbe);              // Toggling a pin

                             /************************************/

/************************************
 * IRQ configure and ISR handling   *
 ************************************/
void GPIO_IRQInterruptConfig (uint8_t IRQNumber , uint8_t EnOrDis) ;
void GPIO_IRQPriorityConfig (uint8_t IRQNumber  , uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* DRIVER_INC_STM32F401XX_GPIO_DRIVERS_H_ */
