/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Ahmed Samir
 */

#include"stm32f401xx_i2c_driver.h"
#include <stddef.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhasewrite(I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead (I2C_RegDef_t *pI2Cx , uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t * pI2CHandle);
static void I2C_MasterHandleTXEInterrupt (I2C_Handle_t * pI2CHandle);




void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// implement the code to disable ITBUFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// implement the code to disable ITEVTEN control Bit
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		pI2CHandle->TxRxState = I2C_READY ;
		pI2CHandle->pRxBuffer = NULL ;
		pI2CHandle->RxLen     = 0 ;


		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
			{
				I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
			}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// implement the code to disable ITBUFEN control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// implement the code to disable ITEVTEN control Bit
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		pI2CHandle->TxRxState = I2C_READY ;
		pI2CHandle->pTxBuffer = NULL ;
		pI2CHandle->TxLen     = 0 ;

}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx ,uint8_t EnorDis)
{
	if(EnorDis == I2C_ACK_ENABLE)
	{

	    pI2Cx->CR1 |= (1 << I2C_CR1_ACK) ;
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK) ;
	}
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START) ;
}


 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP) ;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	 uint32_t dummy_read ;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	 {
		 // in master mode
		  if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		   {
			    if(pI2CHandle->RxLen == 1 )
			     {
				   // fist disable the ACK
				      I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				   // clear ADDR flag (read SR1 , read SR2)

				      dummy_read =pI2CHandle->pI2Cx->SR1 ;
				      dummy_read =pI2CHandle->pI2Cx->SR2 ;
				      (void)dummy_read ;
			      }
		   }
	     else
		   {
				// clear ADDR flag (read SR1 , read SR2)

				   dummy_read =pI2CHandle->pI2Cx->SR1 ;
				   dummy_read =pI2CHandle->pI2Cx->SR2 ;
				   (void)dummy_read ;
		   }

	  }
	else
	 {
		// device in a slave mode
		// clear ADDR flag (read SR1 , read SR2)
						   dummy_read =pI2CHandle->pI2Cx->SR1 ;
						   dummy_read =pI2CHandle->pI2Cx->SR2 ;
						   (void)dummy_read ;
	 }

}
  static void I2C_ExecuteAddressPhasewrite(I2C_RegDef_t *pI2Cx , uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1 ;
	slaveAddr &= ~(1);
	pI2Cx->DR = slaveAddr ;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx , uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1 ;
	slaveAddr |= 1;
	pI2Cx->DR = slaveAddr ;
}

uint32_t RCC_GetPllOutputClock(void)
{
	return 0;
}


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx ,uint8_t EnorDis)
{
	  if(EnorDis == ENABLE)
	   {
		   //Implement the code to enable ITBUFEN Control Bit
			 pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		   //Implement the code to enable ITEVFEN Control Bit
			 pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		   //Implement the code to enable ITERREN Control Bit
			 pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	   }
	  else
	  {
		  //Implement the code to enable ITBUFEN Control Bit
		 	pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		  //Implement the code to enable ITEVFEN Control Bit
		    pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

		   //Implement the code to enable ITERREN Control Bit
		 	 pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	  }
}


/*****************************
 * peripheral clock setup    *
******************************/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnOrDis)
{
    if(EnOrDis == ENABLE)
    {
    	       if(pI2Cx == I2C1)
    	        {
    		        I2C1_PCLK_EN() ;
    	        }

    	  else if(pI2Cx == I2C2)
    	        {
    	            I2C2_PCLK_EN() ;
		        }

    	  else if(pI2Cx == I2C3)
                {
    	 		    I2C3_PCLK_EN() ;
         	    }
    }
    else
      {
    	     if(pI2Cx == I2C1)
    	       {
      		       I2C1_PCLK_DI() ;
    	       }

    	  else if(pI2Cx == I2C2)
    	       {
     	           I2C2_PCLK_DI() ;
    	       }

         else if(pI2Cx == I2C3)
    	      {
    	    	   I2C3_PCLK_DI() ;
    	      }
      }
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx ,uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pI2Cx->CR1 |=  ( 1 << I2C_CR1_PE ) ;
	}
  else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE ) ;
	}
}

                         /************************************/


/*
 *I2C -initialization
 */

/*****************************************************************
 * @fn               - I2C_init
 *
 * @brief            - this function enables or disables peripheral  clock for the given GPIO port
 *
 * @para[int]         - base address of the I2C peripheral
 *  @para[int]        - ENABLE or DISABLE macros @ stm32f401xx.h
 *  @para[int]
 *
 *  @return           - none
 *
 *  @Note             - none
 *
 */
void I2C_init(I2C_Handle_t *pI2CHandle)
{
    //   first lets configure the I2C_CR1 register
		 uint32_t tempreg = 0 ;
    // enable the clock for the i2c peripheral
		 I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
    //   ACK control bit
		 tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl  << I2C_CR1_ACK ;
         pI2CHandle->pI2Cx->CR1 =  tempreg ;

    //  Configure the FREQ field of CR2
        tempreg = 0 ;
        tempreg  |= RCC_GetPCLK1Value() / 1000000U ;
        pI2CHandle->pI2Cx->CR2 =  ( tempreg & 0x3F) ;


    //  program the device own address
        tempreg = 0 ;
        tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1 ;
        tempreg |= (1 << 14);   // Bit 14 Should always be kept at 1 by software.
        pI2CHandle->pI2Cx->OAR1 =  tempreg ;

   //   CCR calculation
        tempreg = 0 ;
        uint16_t ccr_value = 0 ;

         if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
          {
        	// mode is standard mode
        	   ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
        	   tempreg |= ( ccr_value & 0xFFF ) ;
          }
        else
          {
        	// mode is fast mode
        	 tempreg |= (1 << I2C_CCR_FS) ;
        	 tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY ) ;
        	 if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 )
        	  {
        		 ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
        	  }
        	 else
        	  {
        		 ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ));
        	  }
        	 tempreg |= ( ccr_value & 0xFFF ) ;
          }
       pI2CHandle->pI2Cx->CCR |=  tempreg ;


       //TRISE configuration
         if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
          {
            	// mode is standard mode
                   tempreg  =(RCC_GetPCLK1Value() / 1000000U) + 1 ;
          }
        else
          {
               // mode is fast mode
                  tempreg  =((RCC_GetPCLK1Value() * 300 )/ 1000000000U) + 1 ;
          }

       pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F) ;

}




/*
 *SPI De-initialization
 */

/*****************************************************************
 * @fn               - I2C_Deinit
 *
 * @brief            - this function enables or disables peripheral  clock for the given GPIO port
 *
 * @para[int]         - base address of the I2C peripheral
 *  @para[int]        - ENABLE or DISABLE macros @ stm32f401xx.h
 *  @para[int]
 *
 *  @return           - none
 *
 *  @Note             - none
 *
 */
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	            if(pI2Cx == I2C1)
	    	     {
	            	RCC->APB2RSTR &= ~(1 << 21);
	    	     }
	        else if(pI2Cx == I2C2)
	             {
	      	        RCC->APB1RSTR &= ~(1 << 22);
         	     }
	       else if(pI2Cx == I2C3)
	             {
	               	RCC->APB1RSTR &= ~(1 << 23);
  	      	     }

}
/**************************************************************************************************************************************/


/*
* @fn               - I2C_GetFlagStatus
*
* @brief            - this function enables or disables peripheral  clock for the given GPIO port
*
* @para[int]         - base address of the I2C peripheral
*  @para[int]        - ENABLE or DISABLE macros @ stm32f401xx.h
*  @para[int]
*
*  @return           - none
*
*  @Note             - none
*
*/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx ,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	 {
		return FLAG_SET ;
	 }

	    return FLAG_RESET ;
}
/**************************************************************************************************************************************/


/*
* @fn               - I2C_MasterSendData
*
* @brief            - this function enables or disables peripheral  clock for the given GPIO port
*
* @para[int]         - base address of the I2C peripheral
*  @para[int]        - ENABLE or DISABLE macros @ stm32f401xx.h
*  @para[int]
*
*  @return           - none
*
*  @Note             - none
*
*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t Len ,uint8_t SlaveAddr, uint8_t Sr)
{
	//1.Generate the Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in SR1
       // Note:until SB is cleared SCL Will Be stretched (pulled to LOW)
        while (!  I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_SB) )
        {
        	;
        }

    //3. Send the address of the slave with r/w bit set to w(0)(total 8bits)
        I2C_ExecuteAddressPhasewrite(pI2CHandle->pI2Cx ,SlaveAddr);

    //4. Confirm that address phase is completed by checking the ADDR flag in the SR1
         while (!  I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_ADDR) );

    //5. Clear the ADDR flag according to its software sequence
    //   Note:until ADDR isCleared SCL will be stretched(pulled to zero)
         I2C_ClearADDRFlag(pI2CHandle);

    // 6.Send the data until the len equal zero
     	 while(Len > 0)
     	   {
     		//1. wait until TXE is set
     		  while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE ) )
     			  {;} //wait until TXE is set
     		       pI2CHandle->pI2Cx->DR = *pTxBuffer ;
     		       pTxBuffer ++ ;
     		       Len--;
     	   }
   // 7. when len is equal to zero wait for TXE=1 and BTF=1 before generating the STOP condition
   //    Note:until TXE=1 , BTF=1 ,means that bothSR and DR are empty and next transmission should begin
   //    when  BTF=1 SCL will be stretched(pulled to zero).
     	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE ) );
     	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_BTF ) )
     		{;}


     	   if(Sr == I2C_DISABLE_SR)
     	    {
             //8.Generate the STOP condition
    	       I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
     	    }


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len ,uint8_t SlaveAddr,  uint8_t Sr)
{

	//1.Generate the Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in SR1
	//   Note:until SB is cleared SCL Will Be stretched (pulled to LOW)
		   while (!  I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_SB) )
		      {
		          ;
		      }
	 //3. Send the address of the slave with r/w bit set to w(0)(total 8bits)
		   I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx ,SlaveAddr);

	 //4. Confirm that address phase is completed by checking the ADDR flag in the SR1
		         while (!  I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_FLAG_ADDR) );

	//5. procedure to read only 1 byte from slave
		if(Len == 1)
		 {
			  // Disable the ACK flag
			         I2C_ManageAcking(pI2CHandle->pI2Cx  , I2C_ACK_DISABLE);

			  // Clear ADDR Flag
			         I2C_ClearADDRFlag(pI2CHandle);

			  // wait until RXNE is set
			       while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_RXNE ) )
			    	   {;}
                     if(Sr == I2C_DISABLE_SR)
                     {
			 // Generate the STOP condition
			       I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                     }
			  // Read the Data

			      	*pRxBuffer = pI2CHandle->pI2Cx->DR ;

		 }
		//5. procedure to read from slave when len >1
				if(Len > 1)
				{

					// Clear ADDR Flag
						 I2C_ClearADDRFlag(pI2CHandle);
				   // read the data until length becomes zero
						 for(uint32_t i = Len ; i > 0 ; i --)
						 {
							 // wait until RXNE is set
							 while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_RXNE ) );
                             if(i == 2)
                              {
                            	 // Disable the ACK flag
                            	    I2C_ManageAcking(pI2CHandle->pI2Cx  , I2C_ACK_DISABLE);

                            	    if(Sr == I2C_DISABLE_SR)
                            	     {
                            		 // Generate the STOP condition
                            		    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                            	     }
                              }
                        // Read the Data
                           	*pRxBuffer = pI2CHandle->pI2Cx->DR ;
                       // increment the buffer address
                           	pRxBuffer ++ ;

						 }

				 }


				 // Enable the ACK flag
				if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
				{
			     I2C_ManageAcking(pI2CHandle->pI2Cx  , I2C_ACK_ENABLE);
				}
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState ;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen     = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr   = SlaveAddr ;
		pI2CHandle->Sr        = Sr;


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	}

	return busystate;

}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState ;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen     = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr   = SlaveAddr ;
		pI2CHandle->Sr        = Sr;



		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	}

	return busystate;

}
void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{// make sure that TXE is also set
    if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
     {
	   //BTF, TXE = 1
         if(pI2CHandle->TxLen == 0)
          {
	   //1. generate the STOP condition
	        if(pI2CHandle->Sr == I2C_DISABLE_SR);
	        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	   //2. reset all the elements of the handle structure.
            I2C_CloseSendData(pI2CHandle);
	   //3.notify the application about transmission complete.
	       I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
          }
                 if(pI2CHandle->TxLen > 0 )
                  {
        	    			  //1. load the data in to DR
        	    			       pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        	    			  //2. decrement the TxLen
        	    			       pI2CHandle->TxLen -- ;
        	    			  //3. increment the buffer address
        	    			       pI2CHandle->pTxBuffer ++ ;
                   }
      }
}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//we have to do the data reception
		  			   if(pI2CHandle->RxLen == 1)
		  			    {
		  				      *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		  				       pI2CHandle->RxLen -- ;
		  			    }
		  		       if(pI2CHandle->RxLen > 1)
		  			    {
		  			             if(pI2CHandle->RxLen == 2)
		  			              {
		  			                  // Disable the ACK flag
		  			                     I2C_ManageAcking(pI2CHandle->pI2Cx  , I2C_ACK_DISABLE);
		  			              }
		  			          // Read the Data
		  			             *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		  			          // increment the buffer address
		  			              pI2CHandle->pRxBuffer ++ ;
		  			              pI2CHandle->RxLen --     ;
		  			      }

		  		        if(pI2CHandle->RxLen == 0)
		  		   	     {
		  			          // close the I2C data reception and notify the application

		  			           //1.generate stop condition
		  			                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		  			           //2. close the T2C rx
		  			                I2C_CloseReceiveData(pI2CHandle);

		  			          //3. Notify the application
		  			                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
		  		   	     }
}
void I2C_EV_IRQHandling     (I2C_Handle_t *pI2CHandle)
{
	// interrupt handling for both master and slave mode of a device

	uint32_t temp1 , temp2 ,temp3 ;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);


	//1. Handle for interrupt generated by SB event
	// Note : SB flag is only applicable in Master Mode .
	   if(temp1 && temp3)
	   {
		    // The interrupt is  generated because of SB event.
		    // This block will not be executed in slave mode because for slave SB is always zero.
		    // In this block lets execute the address phase
		   if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		    {
			   I2C_ExecuteAddressPhasewrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		    }
	  else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		   {
		         I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		   }
	   }

	 temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	//2. Handle for interrupt generated by ADDR event
	// Note : when master mode -->address is sent
	//      : when slave  mode -->address matched with own address
	   if(temp1 && temp3)
		{
			//ADDR flag is set
		   I2C_ClearADDRFlag(pI2CHandle);
		}

	 temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	 //3. Handle for interrupt generated by BTF (Byte transfer finished) event.
	    if(temp1 && temp3)
		 {
				//BTF flag is set
	    	 if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    	  {
	    		   // make sure that TXE is also set.
	    		 if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE))
	    		  {
	    		 	    		 //  TXE ,BTF are  set.
	    			   if(pI2CHandle->TxLen == 0)
	    			   {

	    			    // 1.generate the stop condition
	    			         if(pI2CHandle->Sr == I2C_DISABLE_SR)
	    			          {
	    			         // Generate the STOP condition
	    			            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	    			          }

	    			 // 2. reset all the member elements of the handle structure
	    			         I2C_CloseSendData(pI2CHandle);
	    			 // 3. Notify the application about transmission complete
	    		             I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

	    		        }
	    		  }
	          }
         else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	          {
                   ;
	    	  }

		}

	  temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	  //4. Handle for interrupt generated by STOPF event.
      //   Note : Stop detection flag is applicable only on slave mode .for master this flag never set
	  //         The below code block will not be executed by the master since STOPF will not set in master mode
	      if(temp1 && temp3)
	       {
				//STOP flag is set
	    	    //Clear the sTOPF (i.e )read SR1 then write to CR1
	    	      pI2CHandle->pI2Cx->CR1 |=0x0000 ;

	    	  //3.notify the application about transmission complete.
		             I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
		   }

	  temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	  //5.  Handle for interrupt generated by TXE event.
 if(temp1 && temp2 && temp3)
	{
	   // check for the device mode
	     if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	     {
	    	 // master mode
	    		//TXE flag is set
	    	    //we have to do data transmission
	    	  if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    	  {
	    		  I2C_MasterHandleTXEInterrupt(pI2CHandle);
	    	  }
	     }
	   else
	     {
		     // slave mode
		     //make sure that the slave is really in transmitter mode
		       if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
		        {
		            I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		        }
	     }
    }


	  temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6.  Handle for interrupt generated by  RXNE event.
	  	 if(temp1 && temp2 && temp3)
	  	  {
	  		    // check for the device mode
	  			   if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	  			    {
	  		         //the device is master
	  		         //RXNE flag is set
	  		           if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	  		            {
	  			            I2C_MasterHandleRXNEInterrupt(pI2CHandle);
	  	                }
	  			   }
	  			 else
	  			  {
	  					 // slave mode
	  					 // make sure that the slave is really in transmitter mode
	  					    if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
	  					     {
	  					        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
	  					     }
	  		     }

	  	  }

}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		  pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		  I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/under run error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		  pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		  I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
void I2C_SlaveSendData   (I2C_RegDef_t *pI2C, uint8_t data)
{
	 pI2C->DR = data ;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C              )
{
	return 	 (uint8_t) pI2C->DR;
}





void I2C_IRQInterruptConfig (uint8_t IRQNumber , uint8_t  EnOrDis)
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
void I2C_IRQPriorityConfig  (uint8_t IRQNumber , uint32_t IRQPriority)
{
	 uint8_t iprx = IRQNumber / 4 ;
	 uint8_t iprx_section =IRQNumber % 4  ;
     uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
 	 *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount );
}
