/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Ahmed Samir
 */



#include"stm32f401xx_gpio_driver.h"

/*                    ************************************************
                      *   APIs supported by this driver               *
                      ************************************************           */

/*
 * 1-peripheral clock setup
 */

/*****************************************************************
 * @fn               - GPIO_PeriClockControl
 *
 * @brief            - this function enables or disables peripheral  clock for the given GPIO port
 *
 * @para[int]         - base address of the gpio peripheral
 *  @para[int]        - ENABLE or DISABLE macroa @ stm32f401xx.h
 *  @para[int]
 *
 *  @return           - none
 *
 *  @Note             - none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnOrDis)
{
    if(EnOrDis == ENABLE)
    {
    	    if(pGPIOx == GPIOA)
    	     {
    		    GPIOA_PCLK_EN() ;
    	     }

    	 else if(pGPIOx == GPIOB)      /* need to be completed   for other ports*/
		      {
    		     GPIOB_PCLK_EN() ;
		      }

         else if(pGPIOx == GPIOC)       /* need to be completed   for other ports*/
    		  {
        		 GPIOC_PCLK_EN() ;
    		  }

         else if(pGPIOx == GPIOD)       /* need to be completed   for other ports*/
        	  {
            	 GPIOD_PCLK_EN() ;
        	  }

         else if(pGPIOx == GPIOE)       /* need to be completed   for other ports*/
           	  {
                 GPIOE_PCLK_EN() ;
           	  }
         else if(pGPIOx == GPIOH)       /* need to be completed   for other ports*/
              {
                 GPIOH_PCLK_EN() ;
              }
     }

    else
      {

    	     if(pGPIOx == GPIOA)
    	      {
    		       GPIOA_PCLK_DI()  ;
    	      }

    	 else if((pGPIOx == GPIOB))       /* need to be completed   for other ports*/
    		  {
    	    		GPIOB_PCLK_DI() ;
    		  }

    	 else if(pGPIOx == GPIOC)       /* need to be completed   for other ports*/
    	      {
    	       	    GPIOC_PCLK_DI() ;
    	      }

    	 else if(pGPIOx == GPIOD)       /* need to be completed   for other ports*/
    	       {
    	       	    GPIOD_PCLK_DI() ;
    	       }

    	 else if(pGPIOx == GPIOD)       /* need to be completed   for other ports*/
    	       {
    	             GPIOE_PCLK_DI() ;
    	       }

    	  else if(pGPIOx == GPIOH)       /* need to be completed   for other ports*/
    	       {
    	             GPIOH_PCLK_DI() ;
    	       }
       }

}

/*
 * peripheral initialization process
 */

/*****************************************************************
 * @fn               - GPIO_PeriClockControl
 *
 * @brief            - this function enables or disables peripheral  clock for the given GPIO port
 *
 * @para[int]         - base address of the gpio peripheral
 *  @para[int]        - ENABLE or DISABLE macroa @ stm32f401xx.h
 *  @para[int]
 *
 *  @return           - none
 *
 *  @Note             - none
 *
 */

void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{

 uint32_t temp = 0 ;

	  // enable GPIO peripheral clock
	     GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

      //1 . configure the mode of GPIO pin
	  if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
       {
		     temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		     pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		     pGPIOHandle->pGPIOx->MODER |= temp;
       }

	  else
	   {
		      if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		        {
		           //1a*configure the FTSR
		    	        EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		    	        EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
         		    	EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		        }

	        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		        {
	               //1b*configure the RSTR
	    	         EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	         EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	         EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		        }

	       else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		   	   {
	               //1c*configure both the FTSR and the RSTR
	    	        EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	        EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	        EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	        EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	    	    }
                   //2* configure the GPIO port selection in syscfg_exit
                    uint8_t temp1 =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) / 4 ;
                    uint8_t temp2 =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) % 4 ;
                    uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
                    SYSCFG_PCLK_EN() ;
                    SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4) ;

		          //3* Enable the EXTI interrupt delivery using IMR
		            EXTI->IMR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		   	        EXTI->IMR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	   }

	temp = 0 ;

//2 . configure the speed
	          temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	          pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	          pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	          temp = 0 ;

//3 . configure the PUPD settings.
	          temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	          pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	          pGPIOHandle->pGPIOx->PUPDR |= temp;

	          temp = 0 ;

//4 . configure the out put type (OPTYP).
	           temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	           pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	           pGPIOHandle->pGPIOx->OTYPER |= temp;

	           temp = 0 ;

//5 . configure the alt functionality.
	           if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
	            {
	                //configure the alt register
	                	uint8_t temp1 , temp2;

	                   temp1 =pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber / 8 ;
	                   temp2 =pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber % 8 ;
	                   pGPIOHandle->pGPIOx->_AFR[temp1] &= ~ (0xF <<(4 * temp2));
	                   pGPIOHandle->pGPIOx->_AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2));


	            }
}

/*
 * GPIO De-initialization
 */

/*****************************************************************
 * @fn               - GPIO_Deinit
 *
 * @brief            - this function enables or disables peripheral  clock for the given GPIO port
 *
 * @para[int]         - base address of the gpio peripheral
 *  @para[int]        - ENABLE or DISABLE macroa @ stm32f401xx.h
 *  @para[int]
 *
 *  @return           - none
 *
 *  @Note             - none
 *
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	            if(pGPIOx == GPIOA)
	    	     {
	            	     RCC->AHB1RSTR &= ~(1 << (0));
	    	     }
	            else if(pGPIOx == GPIOB)
	         	 {
	         	         RCC->AHB1RSTR &= ~(1 << (1));
	         	 }
	            else if(pGPIOx == GPIOC)
	           	 {
	           	         RCC->AHB1RSTR &= ~(1 << (2));
	           	 }
	            else if(pGPIOx == GPIOD)
	           	 {
	           	        RCC->AHB1RSTR &= ~(1 << (3));
	           	 }
	           else if(pGPIOx == GPIOE)
	           	 {
	           	         RCC->AHB1RSTR &= ~(1 << (4));
	           	 }
	           else if(pGPIOx == GPIOH)
	          	 {
	          	       RCC->AHB1RSTR &= ~(1 << (7));
	          	 }

}
/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
       uint8_t value;
          value = (uint8_t)(( pGPIOx->IDR >>PinNumber ) & 0x00000001) ;
          return value;

}


uint16_t  GPIO_ReadFromInputport(GPIO_RegDef_t *pGPIOx)
{
      uint8_t value;
	       value = (uint16_t)( pGPIOx->IDR);
	       return value;
}
void GPIO_writeToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber  ,uint8_t Value)
{
      if (Value == GPIO_PIN_SET)
       {
    	 pGPIOx->ODR |= (1<<(PinNumber));
       }
     else if (Value == GPIO_PIN_RESET)
          {
         	 pGPIOx->ODR &=~ (1<<(PinNumber));
          }

}
void GPIO_writeToOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
	pGPIOx->ODR = Value ;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	pGPIOx->ODR  ^= (1<<PinNumber) ;
}


/*
 * IRQ config and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnOrDis)
{

	if(EnOrDis == ENABLE)
	{
		  if(IRQNumber <= 31)
		   {
			   *NVIC_ISER0 |=  (1 << IRQNumber ) ;
		   }

	   else if(IRQNumber > 31 && IRQNumber < 64)
		    {
		       *NVIC_ISER1  |=  (1 << (IRQNumber % 32 ) ) ;
		    }

	   else if(IRQNumber >= 64 && IRQNumber < 96)
		    {
		      *NVIC_ISER2  |=  (1 << (IRQNumber % 64 ) ) ;
		    }
	}

	else
	{
		     if(IRQNumber <= 31)
			  {
		    	 *NVIC_ICER0 |=  (1 << IRQNumber ) ;
		   	  }

		else if(IRQNumber > 31 && IRQNumber < 64)
			  {
			    *NVIC_ICER1  |=  (1 << (IRQNumber % 32 ) ) ;
     	      }

	    else if(IRQNumber >= 64 && IRQNumber < 96)
			 {
	    	   *NVIC_ICER2  |=  (1 << (IRQNumber % 64 ) ) ;
			 }
	}



}

void GPIO_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority)
{
  	uint8_t iprx = IRQNumber / 4 ;
  	uint8_t iprx_section =IRQNumber % 4  ;
  	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  	 *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
}

void  GPIO_IRQHandling (uint8_t PinNumber)
{
  //clear the exti register corresponding to the pin

	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |=(1 << PinNumber);
	}

}


