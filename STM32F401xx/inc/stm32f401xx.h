/*
 * stm32f401xx.h
 *
 *  Created on: Oct 31, 2023
 *      Author: Ahmed samir
 */

#ifndef DRIVERS_INC_STM32F401XX_H_
#define DRIVERS_INC_STM32F401XX_H_

#include<stdint.h>

#define __vo volatile



/*                    *******************************************************
                      *  ARM Cotrex Mx processor NVIC_ISERx Register address  *
                      *******************************************************                      */
#define NVIC_ISER0      (( __vo uint32_t *)0xE000E100  )
#define NVIC_ISER1      (( __vo uint32_t *)0xE000E104  )
#define NVIC_ISER2      (( __vo uint32_t *)0xE000E108  )
#define NVIC_ISER3      (( __vo uint32_t *)0xE000E10C  )



/*                    *******************************************************
                      *  ARM Cotrex Mx processor NVIC_ICERx Register address  *
                      *******************************************************                      */
#define NVIC_ICER0     (( __vo uint32_t *)0XE000E180   )
#define NVIC_ICER1     (( __vo uint32_t *)0XE000E184   )
#define NVIC_ICER2     (( __vo uint32_t *)0XE000E188   )
#define NVIC_ICER3     (( __vo uint32_t *)0XE000E18C   )


/*                    *******************************************************
                      *  ARM Cotrex Mx processor NVIC_IPRx Register address  *
                      *******************************************************                      */
#define NVIC_PR_BASE_ADDR      (( __vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED      4

/*                    *****************************************
                      *  Flash, ROM and SRAM  base addresses    *
                      *****************************************                      */

#define FLASH_BASEADDR       0x08000000U               /*the base address of the flash memory with 512 Kbytes */
#define SRAM_BASEADDR        0x20000000U               /*the base address of the SRAM with 96 Kbytes */
#define ROM                  0x1FFF0000U               /*the base address of the internal ROM 96 Kbytes */



/*                    ***********************************************
                      *  AHBx , APBx  Bus peripheral base addresses *
                      ***********************************************           */

#define PERIPH_BASE             0x40000000U            /*the base address of the peripherals  */
#define APB1PERIPH_BASE         PERIPH_BASE            /*the base address of the APB1 BUS  */
#define APB2PERIPH_BASE         0x40010000U            /*the base address of the APB2 BUS  */
#define AHB1PERIPH_BASE         0x40020000U            /*the base address of the AHB1 BUS  */
#define AHB2PERIPH_BASE         0x50000000U            /*the base address of the AHB2 BUS  */


/*                    ***********************************************
                      *   Base addresses of peripherals that are    *
                      *   hanging at AHB1 bus                       *
                      ***********************************************           */

#define GPIOA_BASEADDR        ((AHB1PERIPH_BASE) + (0x0000) )          /*the  address of the GPIOA   */
#define GPIOB_BASEADDR        ((AHB1PERIPH_BASE) + (0x0400) )          /*the  address of the GPIOB   */
#define GPIOC_BASEADDR        ((AHB1PERIPH_BASE) + (0x0800) )          /*the  address of the GPIOC   */
#define GPIOD_BASEADDR        ((AHB1PERIPH_BASE) + (0x0C00) )          /*the  address of the GPIOD   */
#define GPIOE_BASEADDR        ((AHB1PERIPH_BASE) + (0x1000) )          /*the  address of the GPIOE   */
#define GPIOH_BASEADDR        ((AHB1PERIPH_BASE) + (0x1C00) )          /*the  address of the GPIOH   */

#define CRC_BASEADDR          ((AHB1PERIPH_BASE) + (0x3000) )          /*the  address of the CRC     */

#define RCC_BASEADDR          ((AHB1PERIPH_BASE) + (0x3800) )          /*the  address of the RCC     */

/*                    ***********************************************
                      *   Base addresses of peripherals that are    *
                      *   hanging at APB1 bus                       *
                      ***********************************************           */

#define I2C1_BASEADDR         ((APB1PERIPH_BASE) + (0x5400) )
#define I2C2_BASEADDR         ((APB1PERIPH_BASE) + (0x5800) )
#define I2C3_BASEADDR         ((APB1PERIPH_BASE) + (0x5C00) )

#define I2S2ext_BASEADDR      ((APB1PERIPH_BASE) + (0x3400) )
#define I2S3ext_BASEADDR      ((APB1PERIPH_BASE) + (0x4000) )

#define IWDG_BASEADDR         ((APB1PERIPH_BASE) + (0x3000) )

#define PWR_BASEADDR          ((APB1PERIPH_BASE) + (0x7000) )

#define RTC_BASEADDR          ((APB1PERIPH_BASE) + (0x2800) )

#define SPI2_BASEADDR         ((APB1PERIPH_BASE) + (0x3800) )
#define SPI3_BASEADDR         ((APB1PERIPH_BASE) + (0x3C00) )


#define TIM2_BASEADDR         ((APB1PERIPH_BASE) + (0x0000) )
#define TIM3_BASEADDR         ((APB1PERIPH_BASE) + (0x0400) )
#define TIM4_BASEADDR         ((APB1PERIPH_BASE) + (0x0800) )
#define TIM5_BASEADDR         ((APB1PERIPH_BASE) + (0x0C00) )

#define USART2_BASEADDR       ((APB1PERIPH_BASE) + (0x4400) )

#define WWDG_BASEADDR         ((APB1PERIPH_BASE) + (0x2C00) )


/*                    ***********************************************
                      *   Base addresses of peripherals that are    *
                      *   hanging at APB2 bus                       *
                      ***********************************************           */

#define ADC1_BASEADDR         ((APB2PERIPH_BASE) + (0x2000) )

#define EXTI_BASEADDR         ((APB2PERIPH_BASE) + (0x3C00) )

#define SDIO_BASEADDR         ((APB2PERIPH_BASE) + (0x2C00) )

#define SPI1_BASEADDR         ((APB2PERIPH_BASE) + (0x3000) )
#define SPI4_BASEADDR         ((APB2PERIPH_BASE) + (0x3400) )

#define SYSCFG_BASEADDR       ((APB2PERIPH_BASE) + (0x3800) )

#define TIM1_BASEADDR         ((APB2PERIPH_BASE) + (0x0000) )
#define TIM9_BASEADDR         ((APB2PERIPH_BASE) + (0x4000) )
#define TIM10_BASEADDR        ((APB2PERIPH_BASE) + (0x4400) )
#define TIM11_BASEADDR        ((APB2PERIPH_BASE) + (0x4800) )

#define USART1_BASEADDR       ((APB2PERIPH_BASE) + (0x1400) )
#define USART6_BASEADDR       ((APB2PERIPH_BASE) + (0x1000) )



/*                    ******************************************************************
                      *   General purpose Peripheral Registers definitions structure     *
                      ******************************************************************             */

typedef struct
{
	__vo uint32_t MODER;          /* GPIO port mode register                 Address offset: 0x00 */
	__vo uint32_t OTYPER;         /* GPIO port output type register          Address offset: 0x04 */
	__vo uint32_t OSPEEDR;        /* GPIO port output speed register         Address offset: 0x08 */
	__vo uint32_t PUPDR;          /* GPIO port pull-up/pull-down register    Address offset: 0x0C */
	__vo uint32_t IDR;            /* GPIO port input data register           Address offset: 0x10 */
	__vo uint32_t ODR;            /* GPIO port output data register          Address offset: 0x14 */
	__vo uint32_t BSRR;           /* GPIO port bit set/reset register        Address offset: 0x18 */
	__vo uint32_t LCKR;           /* GPIO port configuration lock register   Address offset: 0x1C */
	__vo uint32_t _AFR[2];        /* GPIO alternate function low register    Address offset: 0x20 */

}GPIO_RegDef_t ;

/*                    ************************************************
                      *    RCC peripheral registers structure        *
                      ************************************************           */
typedef struct
{
	__vo uint32_t CR;             /* RCC clock control register                  Address offset: 0x00 */
	__vo uint32_t PLLCFGR;        /* RCC PLL configuration register              Address offset: 0x04 */
    __vo uint32_t CFGR;           /* RCC clock configuration register            Address offset: 0x08 */
    __vo uint32_t CIR;            /* RCC clock interrupt register                Address offset: 0x0C */
    __vo uint32_t AHB1RSTR;       /* RCC AHB1 peripheral reset register          Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;       /* RCC AHB2 peripheral reset register          Address offset: 0x14 */
    uint32_t RESERVED0;           /* RESERVED0                                   Address offset: 0x18 */
	uint32_t RESERVED1;           /* RESERVED1                                   Address offset: 0x1C */
    __vo uint32_t APB1RSTR;       /* RCC APB1 peripheral reset register          Address offset: 0x20 */
	__vo uint32_t APB2RSTR;       /* RCC APB2 peripheral clock enable register   Address offset: 0x24 */
    uint32_t RESERVED2;           /* RESERVED2                                   Address offset: 0x28 */
    uint32_t RESERVED3;           /* RESERVED3                                   Address offset: 0x2C */
    __vo uint32_t AHB1ENR;        /* RCC AHB1 peripheral clock enable register   Address offset: 0x30 */
	__vo uint32_t AHB2ENR;        /* RCC AHB2 peripheral clock enable register   Address offset: 0x34 */
    uint32_t RESERVED4;           /* RESERVED4                                   Address offset: 0x38 */
	uint32_t RESERVED5;           /* RESERVED5                                   Address offset: 0x3C */
    __vo uint32_t APB1ENR;        /* RCC APB1 peripheral clock enable register   Address offset: 0x40 */
	__vo uint32_t APB2ENR;        /* RCC APB2 peripheral clock enable register   Address offset: 0x44 */
    uint32_t RESERVED6;           /* RESERVED6                                   Address offset: 0x48 */
	uint32_t RESERVED7;           /* RESERVED7                                   Address offset: 0x4C */
	__vo uint32_t AHB1LPEN;       /* RCC AHB1 peripheral clock enable in low power mode register   Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;      /* RCC AHB2 peripheral clock enable in low power mode register   Address offset: 0x54 */
	uint32_t RESERVED8;           /* RESERVED8                                   Address offset: 0x58 */
	uint32_t RESERVED9;           /* RESERVED9                                   Address offset: 0x5C */
    __vo uint32_t APB1LPENR;      /* RCC APB1 peripheral clock enable in low power mode register   Address offset: 0x60 */
	__vo uint32_t APB2LPENR;      /* RCC APB2 peripheral clock enable in low power mode register   Address offset: 0x64 */
	uint32_t RESERVED10;           /* RESERVED0                                   Address offset: 0x68 */
	uint32_t RESERVED11;           /* RESERVED1                                   Address offset: 0x6C */
    __vo uint32_t BDCR;           /* RCC Backup domain control register             Address offset: 0x70 */
    __vo uint32_t CSR;            /* RCC clock control & status register            Address offset: 0x74 */
    uint32_t RESERVED12;          /* RESERVED12                                     Address offset: 0x78 */
	uint32_t RESERVED13;          /* RESERVED13                                     Address offset: 0x7C */
    __vo uint32_t SSCGR;          /* spread spectrum clock generation register      Address offset: 0x80 */
    __vo uint32_t PLLI2SCFGR;     /* RCC PLLI2S configuration register              Address offset: 0x84 */
    uint32_t RESERVED14;           /* RESERVED14                                    Address offset: 0x88 */
    __vo uint32_t DCKCFGR;        /* RCC Dedicated Clocks Configuration Register    Address offset: 0x8C */

}RCC_RegDef_t ;

/*                    ******************************************************************
                      *   EXTI Peripheral Register definition structure                *
                      ******************************************************************             */


typedef struct
{
	__vo uint32_t IMR;            /* Interrupt mask register                   Address offset: 0x00 */
	__vo uint32_t EMR;            /* Event mask register                       Address offset: 0x04 */
	__vo uint32_t RTSR;           /* Rising trigger selection register         Address offset: 0x08 */
	__vo uint32_t FTSR;           /* Falling trigger selection register        Address offset: 0x0C */
	__vo uint32_t SWIER;          /* Software interrupt event register         Address offset: 0x10 */
	__vo uint32_t PR;             /* Pending register                          Address offset: 0x14 */

}EXTI_RegDef_t ;


/*                    ******************************************************************
                      *  SYSCFG Peripheral Register definition structure                *
                      ******************************************************************             */

typedef struct
{
	__vo uint32_t MEMRMP;            /* SYSCFG memory re map register                               Address offset: 0x00 */
	__vo uint32_t PMC;               /* SYSCFG peripheral mode configuration register               Address offset: 0x04 */
	__vo uint32_t EXTICR[4];         /* SYSCFG external interrupt configuration register 1~4          Address offset: 0x08 */
//	__vo uint32_t EXTICR2;           /* SYSCFG external interrupt configuration register 2          Address offset: 0x0C */
//	__vo uint32_t EXTICR3;           /* SYSCFG external interrupt configuration register 3          Address offset: 0x10 */
//	__vo uint32_t EXTICR4;           /* SYSCFG external interrupt configuration register 4          Address offset: 0x14 */
    uint32_t RESERVED0;              /* RESERVED0                                                   Address offset: 0x18 */
	uint32_t RESERVED1;              /* RESERVED1                                                   Address offset: 0x1C */
	__vo uint32_t CMPCR;             /* Compensation cell control register                          Address offset: 0x20 */

}SYSCFG_RegDef_t ;


/*                    ******************************************************************
                      *   SPI Peripheral Register definition structure                 *
                      ******************************************************************             */
typedef struct
{
	__vo uint32_t CR1;            /* Interrupt mask register                                      Address offset: 0x00 */
	__vo uint32_t CR2;            /* Event mask register                                          Address offset: 0x04 */
	__vo uint32_t SR;             /* Rising trigger selection register                            Address offset: 0x08 */
	__vo uint32_t DR;             /* Falling trigger selection register                           Address offset: 0x0C */
	__vo uint32_t CRCPR;          /* SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)       Address offset: 0x10 */
	__vo uint32_t RXCRCR;         /* SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)       Address offset: 0x14 */
	__vo uint32_t TXCRCR;         /* SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)       Address offset: 0x18 */
	__vo uint32_t I2SCFGR;        /* SPI_I2S configuration register (SPI_I2SCFGR)                 Address offset: 0x1C */
	__vo uint32_t I2SPR;          /* SPI_I2S pre-scaler register (SPI_I2SPR)                      Address offset: 0x20 */

}SPI_RegDef_t ;

/*                    ******************************************************************
                      *   I2C Peripheral Registers definition structure                *
                      ******************************************************************             */
typedef struct
{
	__vo uint32_t CR1;            /* Interrupt mask register                                      Address offset: 0x00 */
	__vo uint32_t CR2;            /* Event mask register                                          Address offset: 0x04 */
	__vo uint32_t OAR1;           /* Rising trigger selection register                            Address offset: 0x08 */
	__vo uint32_t OAR2;           /* Falling trigger selection register                           Address offset: 0x0C */
	__vo uint32_t DR;             /* I2C RX CRC register (I2C_RXCRCR)(not used in I2S mode)       Address offset: 0x10 */
	__vo uint32_t SR1;            /* I2C CRC register (I2C_RXCRCR)(not used in I2S mode)          Address offset: 0x14 */
	__vo uint32_t SR2;            /* I2C CRC register (I2C_TXCRCR)(not used in I2S mode)          Address offset: 0x18 */
	__vo uint32_t CCR;            /* I2C_ configuration register (I2C_I2SCFGR)                    Address offset: 0x1C */
	__vo uint32_t TRISE;          /* I2C_ pre-scaler register (I2C_I2SPR)                         Address offset: 0x20 */
	__vo uint32_t FLTR;           /* I2C_ pre-scaler register (I2C_I2SPR)                         Address offset: 0x20 */

}I2C_RegDef_t ;

/*                    ******************************************************************
                      *   USART Peripheral Registers definition structure                *
                      ******************************************************************             */
typedef struct
{
	__vo uint32_t SR;              /* Status register                            Address offset: 0x00 */
	__vo uint32_t DR;              /* Data register                              Address offset: 0x04 */
	__vo uint32_t BRR;             /* Baud rate register                         Address offset: 0x08 */
	__vo uint32_t CR1;             /* Control register 1                         Address offset: 0x0C */
	__vo uint32_t CR2;             /* Control register 2                         Address offset: 0x10 */
	__vo uint32_t CR3;             /* Control register 3                         Address offset: 0x14 */
	__vo uint32_t GTPR;            /* Guard time and pre-scaler register         Address offset: 0x18 */


}USART_RegDef_t ;


/*
 * clock Enable macros
 */


/*                    ************************************************
                      *    Clock enable macros for GPIOx peripherals *
                      ************************************************          */
#define GPIOA_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 0) )      /* GPIOA peripheral clock enable */
#define GPIOB_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 1) )      /* GPIOB peripheral clock enable */
#define GPIOC_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 2) )      /* GPIOC peripheral clock enable */
#define GPIOD_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 3) )      /* GPIOD peripheral clock enable */
#define GPIOE_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 4) )      /* GPIOE peripheral clock enable */
#define GPIOH_PCLK_EN()     ( RCC->AHB1ENR |= ( 1 << 7) )      /* GPIOH peripheral clock enable */



/*                    ************************************************
                      *    Clock enable macros for I2Cx peripherals *
                      ************************************************          */
#define I2C1_PCLK_EN()        ( RCC->APB1ENR |= ( 1 << 21) )      /* I2C1 peripheral clock enable */
#define I2C2_PCLK_EN()        ( RCC->APB1ENR |= ( 1 << 22) )      /* I2C1 peripheral clock enable */
#define I2C3_PCLK_EN()        ( RCC->APB1ENR |= ( 1 << 23) )      /* I2C1 peripheral clock enable */


/*                    ************************************************
                      *    Clock enable macros for SPIx peripherals *
                      ************************************************          */
#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 12) )        /* SPI1  peripheral clock enable */
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 14) )        /* SPI2  peripheral clock enable */
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 15) )        /* SPI3  peripheral clock enable */
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 13) )        /* SPI4  peripheral clock enable */


/*                    ************************************************
                      *    Clock enable macros for USARTx peripherals *
                      ************************************************          */
#define USART1_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 4 ) )        /* USART1  peripheral clock enable */
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= ( 1 << 17) )        /* USART2  peripheral clock enable */
#define USART6_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 5 ) )        /* USART6  peripheral clock enable */


/*                    ************************************************
                      *    Clock enable macros for SYSCFG peripheral *
                      ************************************************          */

#define SYSCFG_PCLK_EN()      ( RCC->APB2ENR |= ( 1 << 14) )      /* SYSTEM  peripheral clock enable */


/*
 * clock disable macros
 */



/*                    ************************************************
                      *    Clock Disable macros for GPIOx peripherals *
                      ************************************************          */
#define GPIOA_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 0) )      /* GPIO peripheral clock disable */
#define GPIOB_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 1) )      /* GPIO peripheral clock disable */
#define GPIOC_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 2) )      /* GPIO peripheral clock disable */
#define GPIOD_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 3) )      /* GPIO peripheral clock disable */
#define GPIOE_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 4) )      /* GPIO peripheral clock disable */
#define GPIOH_PCLK_DI()     ( RCC->AHB1ENR &= ~( 1 << 7) )      /* GPIO peripheral clock disable */


/*                    ************************************************
                      *    Clock Disable macros for I2Cx peripherals *
                      ************************************************          */
#define I2C1_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 21) )      /* I2C1 peripheral clock disable */
#define I2C2_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 22) )      /* I2C2 peripheral clock disable */
#define I2C3_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 23) )      /* I2C3 peripheral clock disable */

/*                    ************************************************
                      *    Clock Disable macros for SPIx peripherals *
                      ************************************************          */
#define SPI1_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 12) )      /* SPI1  peripheral clock disable */
#define SPI2_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 14) )      /* SPI2  peripheral clock disable */
#define SPI3_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 15) )      /* SPI3  peripheral clock disable */
#define SPI4_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 13) )      /* SPI4  peripheral clock disable */

/*                    ************************************************
                      *    Clock Disable macros for USARTx peripherals *
                      ************************************************          */
#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 4 ) )        /* USART1  peripheral clock enable */
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~( 1 << 17) )        /* USART2  peripheral clock enable */
#define USART6_PCLK_DI()      ( RCC->APB2ENR &= ~( 1 << 5 ) )        /* USART6  peripheral clock enable */


/*                    ************************************************
                      *    Clock Disable macros for SYSCFG peripherals *
                      ************************************************          */
#define SYSCFG_PCLK_DI()      ( RCC->APB2ENR &=~( 1 << 14) )      /* SYSCFG peripheral clock disable */


/*
 * Peripheral definition macros
 */


/*                    ************************************************************
                      *   GPIO,RCC,EXTI and SYSCFG Peripheral definition         *
                      ************************************************************           */
#define GPIOA    ((GPIO_RegDef_t*)   GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)   GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)   GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)   GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)   GPIOE_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)   GPIOH_BASEADDR)

#define RCC      ((RCC_RegDef_t*   )  RCC_BASEADDR   )
#define EXTI     ((EXTI_RegDef_t*  )  EXTI_BASEADDR  )
#define SYSCFG   ((SYSCFG_RegDef_t*)  SYSCFG_BASEADDR)


/*                    ***********************************************
                      *   SPI Peripheral definition                    *
                      ***********************************************           */
#define SPI1     ((SPI_RegDef_t* )SPI1_BASEADDR )
#define SPI2     ((SPI_RegDef_t* )SPI2_BASEADDR )
#define SPI3     ((SPI_RegDef_t* )SPI3_BASEADDR )
#define SPI4     ((SPI_RegDef_t* )SPI4_BASEADDR )


/*                    ***********************************************
                      *   I2C Peripheral definition                    *
                      ***********************************************           */
#define I2C1     ((I2C_RegDef_t* )I2C1_BASEADDR )
#define I2C2     ((I2C_RegDef_t* )I2C2_BASEADDR )
#define I2C3     ((I2C_RegDef_t* )I2C3_BASEADDR )


/*                    ***********************************************
                      *   USART Peripheral definition                    *
                      ***********************************************           */
#define USART1     ((USART_RegDef_t* )USART1_BASEADDR )
#define USART2     ((USART_RegDef_t* )USART2_BASEADDR )
#define USART6     ((USART_RegDef_t* )USART6_BASEADDR )


/*                    ************************************************
                      *           IRQS Numbers macros                *
                      ************************************************          */
#define IRQ_NO_EXTI0           6           /* IRQ_NO_EXTI0 */
#define IRQ_NO_EXTI1           7           /* IRQ_NO_EXTI1 */
#define IRQ_NO_EXTI2           8           /* IRQ_NO_EXTI2 */
#define IRQ_NO_EXTI3           9           /* IRQ_NO_EXTI3 */
#define IRQ_NO_EXTI4           10          /* IRQ_NO_EXTI4 */
#define IRQ_NO_EXTI9_5         23          /* IRQ_NO_EXTI9_5 */
#define IRQ_NO_EXTI15_10       40          /* IRQ_NO_EXTI15_10 */

#define IRQ_NO_SPI1           35           /* IRQ_NO_SPI1 */
#define IRQ_NO_SPI2           36           /* IRQ_NO_SPI2 */
#define IRQ_NO_SPI3           51           /* IRQ_NO_SPI3 */
#define IRQ_NO_SPI4           84           /* IRQ_NO_SPI4 */

#define IRQ_NO_I2C1_EV        31          /* IRQ_NO_I2C1_EV */
#define IRQ_NO_I2C1_ER        32          /* IRQ_NO_I2C1_ER */
#define IRQ_NO_I2C2_EV        33          /* IRQ_NO_I2C2_EV */
#define IRQ_NO_I2C2_ER        34          /* IRQ_NO_I2C2_ER */
#define IRQ_NO_I2C3_EV        72          /* IRQ_NO_I2C3_EV */
#define IRQ_NO_I2C3_ER        73          /* IRQ_NO_I2C3_ER */

#define IRQ_NO_USART1        37          /* IRQ_NO_USART1_EV */
#define IRQ_NO_USART2        38          /* IRQ_NO_USART1_ER */
#define IRQ_NO_USART6        71          /* IRQ_NO_USART2_EV */




/*                    *********************************************************
                      *           IRQS priority values macros                *
                      *********************************************************          */
#define NVIC_IRQ_PRI0          0           /* IRQ_NO_EXTI0 */
#define NVIC_IRQ_PRI1          1           /* IRQ_NO_EXTI1 */
#define NVIC_IRQ_PRI15         15

/*                    ************************************************
                      *           GPIO port codes                    *
                      ************************************************          */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
                                    (x == GPIOB) ? 1 :\
                                    (x == GPIOC) ? 2 :\
                                    (x == GPIOD) ? 3 :\
                                    (x == GPIOE) ? 4 :\
                                    (x == GPIOH) ? 7 :0 )


/*                    ***************************************************************
                      *    Bit position definition of SPI peripheral registers      *
                      ***************************************************************        */
/*
 * Bit position definition SPI_CR1
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15


/*
 * Bit position definition SPI_CR2
 */
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/*
 * Bit position definition SPI_SR
 */
#define SPI_SR_RXNE          0
#define SPI_SR_TXE           1
#define SPI_SR_CHSIDE        2
#define SPI_SR_UDR           3
#define SPI_SR_CRCERR        4
#define SPI_SR_MODF          5
#define SPI_SR_OVR           6
#define SPI_SR_BSY           7
#define SPI_SR_FRE           8


/*                    ***************************************************************
                      *    Bit position definition of I2C peripheral registers      *
                      ***************************************************************        */
/*
 * Bit position definition I2C_CR1
 */
#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRETCH   7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_SWRST       15

/*
 * Bit position definition I2C_CR2
 */
#define I2C_CR2_FREQ         0
#define I2C_CR2_ITERREN      8
#define I2C_CR2_ITEVTEN      9
#define I2C_CR2_ITBUFEN      10
#define I2C_CR2_DMAEN        11
#define I2C_CR2_LAST         12

/*
 * Bit position definition I2C_DR
 */
#define I2C_DR              0

/*
 * Bit position definition I2C_SR1
 */
#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_ADD10       3
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_BERR        8
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_PECERR      12
#define I2C_SR1_TIMEOUT     14
#define I2C_SR1_SMBALERT    15

/*
 * Bit position definition I2C_SR2
 */
#define I2C_SR2_MSL          0
#define I2C_SR2_BUSY         1
#define I2C_SR2_TRA          2
#define I2C_SR2_GENCALL      4
#define I2C_SR2_SMBDEFAULT   5
#define I2C_SR2_SMBHOST      6
#define I2C_SR2_DUALF        7
#define I2C_SR2_PEC          8

/*
 * Bit position definition I2C_CCR
 */
#define I2C_CCR              0
#define I2C_CCR_DUTY         14
#define I2C_CCR_FS           15



/*                    ***************************************************************
                      *    Bit position definition ofUSART peripheral registers      *
                      ***************************************************************        */

/*
 * Bit position definition USART_SR
 */
#define USART_SR_PE          0
#define USART_SR_FE          1
#define USART_SR_NF          2
#define USART_SR_ORE         3
#define USART_SR_IDLE        4
#define USART_SR_RXNE        5
#define USART_SR_TC          6
#define USART_SR_TXE         7
#define USART_SR_LBD         8
#define USART_SR_CTS         9


/*
 * Bit position definition USART_DR
 */

#define USART_BRR_DIV_Fraction          0


/*
 * Bit position definition USART_BRR
 */

#define USART_BRR_DIV_Fraction          0
#define USART_BRR_DIV_Mantissa          4


/*
 * Bit position definition USART_CR1
 */

#define USART_CR1_SBK            0
#define USART_CR1_RWU            1
#define USART_CR1_RE             2
#define USART_CR1_TE             3
#define USART_CR1_IDLEIE         4
#define USART_CR1_RXNEIE         5
#define USART_CR1_TCIE           6
#define USART_CR1_TXEIE          7
#define USART_CR1_PEIE           8
#define USART_CR1_PS             9
#define USART_CR1_PCE            10
#define USART_CR1_WAKE           11
#define USART_CR1_M              12
#define USART_CR1_UE             13
#define USART_CR1_OVER8          15


/*
 * Bit position definition USART_CR2
 */
#define USART_CR2_ADD            0
#define USART_CR2_LBDL           5
#define USART_CR2_LBDIE          6
#define USART_CR2_LBCL           8
#define USART_CR2_CPHA           9
#define USART_CR2_CPOL           10
#define USART_CR2_CLKEN          11
#define USART_CR2_STOP           12
#define USART_CR2_LINEN          14


/*
 * Bit position definition USART_CR3
 */

#define USART_CR3_EIE             0
#define USART_CR3_IREN            1
#define USART_CR3_IRLP            2
#define USART_CR3_HDSEL           3
#define USART_CR3_NACK            4
#define USART_CR3_SCEN            5
#define USART_CR3_DMAR            6
#define USART_CR3_DMAT            7
#define USART_CR3_RTSE            8
#define USART_CR3_CTSE            9
#define USART_CR3_CTSIE           10
#define USART_CR3_ONEBIT          11


/*
 * Bit position definition USART_GTPR
 */

#define USART_BGTPR_DPSC         0
#define USART_GTPR_GT            8



/*                    ************************************************
                      *    Some general Macros                       *
                      ************************************************          */
#define ENABLE             1
#define DISABLE            0
#define SET                ENABLE
#define RESET              DISABLE
#define GPIO_PIN_SET       SET
#define GPIO_PIN_RESET     RESET
#define FLAG_SET           SET
#define FLAG_RESET         RESET




#include"stm32f401xx_gpio_driver.h"
#include"stm32f401xx_spi_driver.h"
#include"stm32f401xx_i2c_driver.h"
#include"stm32f401xx_usart_driver.h"
#include"stm32f401xx_rcc_driver.h"

#endif /* DRIVERS_INC_STM32F401XX_H_ */
