/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Nov 15, 2023
 *      Author: Ahmed Samir
 */

#include"stm32f401xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*****************************
 * peripheral clock setup    *
******************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDis)
{
    if(EnOrDis == ENABLE)
    {
    	       if(pSPIx == SPI1)
    	        {
    		        SPI1_PCLK_EN() ;
    	        }

    	  else if(pSPIx == SPI2)
    	        {
    	            SPI2_PCLK_EN() ;
		        }

    	  else if(pSPIx == SPI3)
                {
    	 		    SPI3_PCLK_EN() ;
         	    }

    	   else if(pSPIx == SPI4)
    	        {
    	            SPI4_PCLK_EN() ;
    	        }

    }


    else
      {

    	     if(pSPIx == SPI1)
    	      {
      		       SPI1_PCLK_DI() ;
    	      }

    	  else if(pSPIx == SPI2)
    	       {
     	           SPI2_PCLK_DI() ;
    	       }

         else if(pSPIx == SPI3)
    	      {
    	    	   SPI3_PCLK_DI() ;
    	      }

         else if(pSPIx == SPI4)
    	     {
    	          SPI4_PCLK_DI() ;
    	     }
    }


}

                         /************************************/


/**********************************
 * initiate and De-init the SPIs *
**********************************/
/*
 * SPI initialization
 */

/*****************************************************************
 * @fn               - GPIO_Deinit
 *
 * @brief            - this function enables or disables peripheral  clock for the given SPI port
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

void SPI_init(SPI_Handle_t *pSPIHandle)
{

	// Peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx , ENABLE);

	//   first lets configure the SPI_CR1 register
		 uint32_t tempreg = 0 ;

    //1. configure the device mode
		 tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;


	//2. configure the bus configuration
          if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
           {
         	   //bidi mode should be cleared
                 tempreg &= ~(1 << SPI_CR1_BIDIMODE ) ;
           }
       else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	        {
    	       //bidi mode should be set
    	         tempreg |= (1 << SPI_CR1_BIDIMODE ) ;
	        }
       else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
  	         {
    	      //bidi mode should be cleared
    	        tempreg &= ~(1 << SPI_CR1_BIDIMODE ) ;
              //RXONLY should be set
            	tempreg |= (1 << SPI_CR1_RXONLY ) ;
  	        }

     //3. configure the SPI serial clock speed (baud rate)
          tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR ;

     //4. configure the DFF
          tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF ;

     //5. configure the CPOL
          tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

     //6. configure the CPHA
          tempreg |= pSPIHandle->SPIConfig.SPI_CPHA  << SPI_CR1_CPHA;

     //7. configure the SSM
          if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN )
          {
        	  tempreg |=  (1<< SPI_CR1_SSM) ;
          }
          else
          {
        	  tempreg &= ~ (1<< SPI_CR1_SSM) ;
          }

      pSPIHandle->pSPIx->CR1 = tempreg ;
}



/*
 *SPI De-initialization
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
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	            if(pSPIx == SPI1)
	    	     {
	            	RCC->APB2RSTR &= ~(1 << 12);
	    	     }
	        else if(pSPIx == SPI2)
	             {
	      	        RCC->APB1RSTR &= ~(1 << 14);
         	     }
	       else if(pSPIx == SPI3)
	             {
	               	RCC->APB1RSTR &= ~(1 << 15);
  	      	     }
	       else if(pSPIx == SPI4)
	            {
	             	RCC->APB2RSTR &= ~(1 << 13);
	    	    }
}

                        /************************************/


/*************************
 * Data Send and Receive  *
 ************************/

/*
 * SPI_SendData
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
 *  @Note             - this is a blocking call
 *
 */

/*
 *SPI_GetFlagStatus
 */


uint8_t SPI_GetFlagStatus (SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	 {
		return FLAG_SET ;
	 }
	return FLAG_RESET ;
}


void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		//1. wait until TXE is set
		          while ( SPI_GetFlagStatus(pSPIx ,SPI_TXE_FLAG ) == FLAG_RESET)
		        	  {
		        	  ;
		        	  }

	   //2. w
		            if(pSPIx->CR1 & (1<< SPI_CR1_DFF))
			         {
				        // 16 bit DFF
		        	       pSPIx->DR = *( (uint16_t*) pTxBuffer);
		        	       len--;
		        	       len--;
		        	       (uint16_t*) pTxBuffer ++ ;
			         }
	              else
		             {
			           // 8 bit DFF
	            	       pSPIx->DR =  *(pTxBuffer);
		                   len--;
		                   pTxBuffer ++ ;
		             }

	 }
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx ,uint8_t EorDis)
{
	if(EorDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnorDis )
{
	if(EnorDis == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}


void SPI_SSOEConfig (SPI_RegDef_t *pSPIx , uint8_t EnorDis )
{
	if(EnorDis == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}


void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len )
{
	while(len > 0)
	{
		//1. wait until RXNE is set
		          while ( SPI_GetFlagStatus(pSPIx ,SPI_RXNE_FLAG ) == FLAG_RESET)
		        	  {
		        	  ;
		        	  }

	   //2. check the data format
		            if(pSPIx->CR1 & (1<< SPI_CR1_DFF))
			         {
				        // 16 bit DFF
		            	*( (uint16_t*) pRxBuffer) = pSPIx->DR ;
		        	       len--;
		        	       len--;
		        	       (uint16_t*) pRxBuffer ++ ;
			         }
	              else
		             {
			           // 8 bit DFF
	            	  *(pRxBuffer) = pSPIx->DR ;
		                   len--;
		                   pRxBuffer ++ ;
		             }

	 }
}


uint8_t SPI_SendDataIT    (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len )
{
	uint8_t state = pSPIHandle->TxState ;

	if(state !=SPI_BUSY_IN_TX)
	{
	//1. save the Tx buffer address and len information in some global variables
         pSPIHandle->pTxBuffer = pTxBuffer ;
         pSPIHandle->TxLen = len ;
     //2. mark the SPI state as busy in transmission so that
     // no other code can take over same SPI peripheral until transmission is over
         pSPIHandle->TxState = SPI_BUSY_IN_TX;
    //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
         pSPIHandle->pSPIx->CR2  |= (1 << SPI_CR2_TXEIE) ;

    //4 . Data transmission will be handled by the ISR code


	}

	return state ;

}
uint8_t SPI_ReceiveDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len )
{
	uint8_t state = pSPIHandle->RxState ;

	if(state !=SPI_BUSY_IN_RX)
	{
	//1. save the Rx buffer address and len information in some global variables
         pSPIHandle->pRxBuffer = pRxBuffer ;
         pSPIHandle->RxLen = len ;
     //2. mark the SPI state as busy in transmission so that
     // no other code can take over same SPI peripheral until transmission is over
         pSPIHandle->RxState = SPI_BUSY_IN_RX;
    //3. Enable the RXNEIE control bit to get interrupt whenever RXN E flag is set in SR
         pSPIHandle->pSPIx->CR2  |= (1 << SPI_CR2_RXNEIE) ;

    //4 . Data transmission will be handled by the ISR code


	}

	return state ;

}



/************************************
 * IRQ configure and ISR handling   *
 ************************************/
void SPI_IRQInterruptConfig (uint8_t IRQNumber , uint8_t EnOrDis)
{
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

}
void SPI_IRQPriorityConfig (uint8_t IRQNumber  , uint32_t IRQPriority)
{
	{
	  	uint8_t iprx = IRQNumber / 4 ;
	  	uint8_t iprx_section =IRQNumber % 4  ;
	  	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	  	 *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

			            if(pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF))
				         {
					        // 16 bit DFF
			            	pSPIHandle->pSPIx->DR = *( (uint16_t*) pSPIHandle->pTxBuffer);
			            	pSPIHandle->TxLen --;
			            	pSPIHandle->TxLen --;
			        	    (uint16_t*) pSPIHandle->pTxBuffer ++ ;
				         }
		              else
			             {
				            // 8 bit DFF
		            	    pSPIHandle->pSPIx->DR =  *(pSPIHandle->pTxBuffer );
		            	    pSPIHandle->TxLen --;
		            	    pSPIHandle->pTxBuffer ++ ;
			             }
			            if(! pSPIHandle->TxLen )
			            {
			            	// TxLen is Zero , close the spi transmission and inform the application that
			            	//Tx is over.

			            	// this prevents interrupts from setting up of TXE flag
			            	SPI_CloseTransmission (pSPIHandle);
			            	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_TX_CMPLT);

			            }


}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

      if(pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF))
       {
          // 16 bit DFF
    	     *( (uint16_t*) pSPIHandle->pRxBuffer) =(uint16_t)pSPIHandle->pSPIx->DR  ;
    	     pSPIHandle->RxLen --;
    	     pSPIHandle->RxLen --;
	         pSPIHandle->pRxBuffer ++ ;
	         pSPIHandle->pRxBuffer ++ ;
      }
    else
     {
        // 8 bit DFF
    	   *(pSPIHandle->pRxBuffer)  = (uint8_t)pSPIHandle->pSPIx->DR ;
	       pSPIHandle->RxLen --;
	       pSPIHandle->pRxBuffer ++ ;
     }
    if(! pSPIHandle->RxLen )
     {
    	  // TxLen is Zero , close the spi transmission and inform the application that
    	  //Tx is over.

    	   // this prevents interrupts from setting up of RXNE flag
    	    SPI_CloseReception(pSPIHandle);
    	    SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_RX_CMPLT);

    }
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{


	//1. clear the ovr flag
	     if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	     {
	    	 uint8_t temp ;
	    	 temp = pSPIHandle->pSPIx->DR ;
	    	 temp = pSPIHandle->pSPIx->SR ;
	    	 (void)temp;

	     }

	//2.
	     SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling (SPI_Handle_t *pHandle )
{
	uint8_t temp1 ,temp2 ;
	// first lets check TXE
	temp1 = pHandle ->pSPIx->SR &(1 << SPI_SR_TXE );
	temp2 = pHandle ->pSPIx->CR2 &(1 << SPI_CR2_TXEIE );

	 if(temp1 && temp2)
	  {
		  spi_txe_interrupt_handle(pHandle);
	  }

	// second lets check RXNE
		temp1 = pHandle ->pSPIx->SR &(1 << SPI_SR_RXNE );
		temp2 = pHandle ->pSPIx->CR2 &(1 << SPI_CR2_RXNEIE );

	 if(temp1 && temp2)
	  {
			spi_rxne_interrupt_handle(pHandle);
	  }

	 // third lets check OVR
	 		temp1 = pHandle ->pSPIx->SR &(1 << SPI_SR_OVR );
	 		temp2 = pHandle ->pSPIx->CR2 &(1 << SPI_CR2_ERRIE );
	  if(temp1 && temp2)
	   {
	 		spi_ovr_err_interrupt_handle(pHandle);
	   }

}
void SPI_ClearOVRFlag (SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR ;
	temp = pSPIx->SR ;
	(void)temp ;
}
void SPI_CloseTransmission (SPI_Handle_t * pSPIHandle)
{
	 pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_TXEIE);
	 pSPIHandle->pTxBuffer  =NULL ;
	 pSPIHandle->TxLen = 0 ;
	 pSPIHandle->TxState = SPI_READY ;

}
void SPI_CloseReception    (SPI_Handle_t * pSPIHandle)
{
	 pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_RXNEIE);
     pSPIHandle->pRxBuffer  =NULL ;
	 pSPIHandle->RxLen = 0 ;
     pSPIHandle->RxState = SPI_READY ;

}
__attribute__ ((weak))void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle , uint8_t AppEv)
{

}
