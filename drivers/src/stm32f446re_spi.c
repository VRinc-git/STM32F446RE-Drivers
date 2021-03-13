/*
 * stm32f446re_spi.c
 *
 *  Created on: Feb 28, 2021
 *      Author: tamor
 */

#include "stm32f446re_spi.h"



static void SPI_TXE_Interrupt_Handle();
static void SPI_RXNE_Interrupt_Handle();
static void SPI_OVR_Interrupt_Handle();


/*************************** SPIx Driver support APIs ******************************/



/******************************************************************************
 * @fn					- SPIx Peripheral clock Control
 *
 * @brief				- Enable or Disable the clock for SPIx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- ENABLE or DISABLE
 *
 * @return				- void
 *
 * @note				-


 */

void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}



/*
 * Init and DeInit
 */

/******************************************************************************
 * @fn					- SPIx Peripheral Initialization
 *
 * @brief				- Initialize the SPIx peripherals
 *
 * @params				- pointer to the Handle
 *
 * @return				- void
 *
 * @note				-


 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	// 1. Device mode configuration
	tempreg |= (pSPIHandle->Init.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. Serial clock baud rate
	tempreg |= (pSPIHandle->Init.SPI_BaudRate << SPI_CR1_BR);

	// 3. CPOL and CPHA
	tempreg |= (pSPIHandle->Init.SPI_ClockPolarity << SPI_CR1_CPOL);
	tempreg |= (pSPIHandle->Init.SPI_ClockPhase << SPI_CR1_CPHA);

	// 4. Bus Configuration
	if(pSPIHandle->Init.SPI_BusConfig == SPI_BUS_FD)
	{
		// 2 line communication - Unidirectional data mode selected
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->Init.SPI_BusConfig == SPI_BUS_HD)
	{
		// 1 line communication - Bidirectional data mode selected
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->Init.SPI_BusConfig == SPI_BUS_SIMPLEX_RX_ONLY)
	{
		// 2 line communication - Unidirectional data mode selected
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);

		// Setting RXONLY to make it receive only
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}

	// 5. Data frame format
	tempreg |= (pSPIHandle->Init.SPI_DataFrameFormat << SPI_CR1_DFF);

	// 6. Slave management
	tempreg |= (pSPIHandle->Init.SPI_SlaveManagement << SPI_CR1_SSM);
	// if Software slave management is enabled the SSI should be set
	if(pSPIHandle->Init.SPI_SlaveManagement == SPI_SLAVE_MANAGEMENT_SW)
	{
		tempreg |= (1 << SPI_CR1_SSI);
	}

	pSPIHandle->pSPIx->CR1 = tempreg;
}



/******************************************************************************
 * @fn					- SPIx Peripheral Deinitialization
 *
 * @brief				- Deinitialize the SPIx peripherals
 *
 * @params				- pointer to the register definition
 *
 * @return				- void
 *
 * @note				-


 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}



/*
 * Data send and receive
 */

/******************************************************************************
 * @fn					- SPIx Send Data
 *
 * @brief				- Sending data form the SPIx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- pointer to the transfer buffer
 * @params				- length of the data to be sent
 *
 * @return				- void
 *
 * @note				-Blocking call


 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
	/* 1. Checking whether the length is 0 or not.
	 * if 0 then block until the length become 0
	 */
	while(length > 0)
	{
		/* 2. check whether the TxBuffer is empty or not
		 * by checking the status of the TXE Flag in SPI_SR register
		 * if not then block until the flag is set.
		 */
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == RESET);

		// 3. Check for the data frame format whether it is 8bit or 16bit

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// if DFF is 16bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);		// writing the data to the data register
			length--;									// decrementing the length two times
			length--;
			(uint16_t*)pTxBuffer++;						// Incrementing TxBuffer
		}
		else
		{
			// if DFF is 8bit
			pSPIx->DR = *(pTxBuffer);					// writing the data to the data register
			length--;									// decrementing the length
			pTxBuffer++;								// Incrementing TxBuffer

		}
	}

}


/******************************************************************************
 * @fn					- SPIx Receive Data
 *
 * @brief				- Receiving data form the SPIx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- pointer to the Receive buffer
 * @params				- length of the data to be received
 *
 * @return				- void
 *
 * @note				-


 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
	/* 1. Checking whether the length is 0 or not.
	 * if 0 then block until the length become 0
	 */
	while(length > 0)
	{
		/* 2. check whether the RxBuffer is non-empty or not
		 * by checking the status of the RXNE Flag in SPI_SR register
		 * if not then block until the flag is set.
		 */
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == RESET);

		// 3. Check for the data frame format whether it is 8bit or 16bit

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// if DFF is 16bit
			*((uint16_t*)pRxBuffer) = pSPIx->DR;		// reading data from the data register
			length--;									// decrementing the length two times
			length--;
			(uint16_t*)pRxBuffer++;						// Incrementing RxBuffer
		}
		else
		{
			// if DFF is 8bit
			*(pRxBuffer) = pSPIx->DR;					// reading data to the data register
			length--;									// decrementing the length
			pRxBuffer++;								// Incrementing RxBuffer

		}
	}
}


/******************************************************************************
 * @fn					- SPIx Send Data with interrupt
 *
 * @brief				- Sending data to the SPIx peripherals when interrupt occured
 *
 * @params				- pointer to the Handle structure
 * @params				- pointer to the Transmit buffer
 * @params				- length of the data to be transmited
 *
 * @return				- SPI application state
 *
 * @note				- Non blocking call


 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{
	// Check the current state of the SPI peripheral
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_STATE_BUSY_IN_TX)
	{
		// 1. Save the TxBuffer address and length info in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLength = length;

		// 2. Mark the SPI state as Busy
		pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt when the TXE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data transmission will be handled by the ISR
	}

	return state;
}



/******************************************************************************
 * @fn					- SPIx Receive Data with interrupt
 *
 * @brief				- receiving data from the SPIx peripherals when interrupt occured
 *
 * @params				- pointer to the Handle structure
 * @params				- pointer to the Receive buffer
 * @params				- length of the data to be received
 *
 * @return				- SPI application state
 *
 * @note				- Non blocking call


 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{
	// Check the current state of the SPI peripheral
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_STATE_BUSY_IN_RX)
	{
		// 1. Save the RxBuffer address and length info in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLength = length;

		// 2. Mark the SPI state as Busy
		pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt when the RXNE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Reception will be handled by the ISR

	}
	return state;
}


/*
 * Interrupt Handle
 */

/******************************************************************************
 * @fn					- SPIx IRQ Handler
 *
 * @brief				- Handle the IRQ of the SPIx peripherals
 *
 * @params				- pointer to the Handle structure
 *
 * @return				- void
 *
 * @note				-


 */
void SPI_IRQHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t state1, state2;

	// 1. check for TXE flag and TXEIE bit
	state1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	state2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(state1 && state2)
	{
		//txe handle
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	// 2. check for RXNE flag and RXNEIE bit
	state1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	state2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(state1 && state2)
	{
		//rxne handle
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	// 3. check for OVR flag and ERRIE bit
	state1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	state2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(state1 && state2)
	{
		//txe handle
		SPI_OVR_Interrupt_Handle(pSPIHandle);
	}
}


/************************************* other functions ***************************************/

/******************************************************************************
 * @fn					- SPIx Get Flag Status
 *
 * @brief				- Return the status of the flags of SPIx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- status of the flag to be checked
 *
 * @return				- Flag status
 *
 * @note				-


 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag)
{
	if(pSPIx->SR & Flag)
	{
		return SET;
	}
	return RESET;
}


/******************************************************************************
 * @fn					- SPIx peripheral Control
 *
 * @brief				- Enable or disable the SPIx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- ENABLE or DISABLE
 *
 * @return				- void
 *
 * @note				-


 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/******************************************************************************
 * @fn					- SPIx TXE Interrupt Handler
 *
 * @brief				- Handle the data when TXE interrupt is occured
 *
 * @params				- pointer to the Handle structure
 *
 * @return				- void
 *
 * @note				-


 */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// 1. Check for the data frame format whether it is 8bit or 16bit

		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// if DFF is 16bit
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);		// writing the data to the data register
			pSPIHandle->TxLength--;												// decrementing the length two times
			pSPIHandle->TxLength--;
			(uint16_t*)pSPIHandle->pTxBuffer++;									// Incrementing TxBuffer
		}
		else
		{
			// if DFF is 8bit
			pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);					// writing the data to the data register
			pSPIHandle->TxLength--;												// decrementing the length
			pSPIHandle->pTxBuffer++;														// Incrementing TxBuffer

		}

		// Once everything is completed check whether the length is 0 or not
		// if so, then end the transmission
		if(!pSPIHandle->TxLength)
		{
			// 1. reset the TXEIE
			pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

			// 2. reset the TxBuffer
			pSPIHandle->pTxBuffer = NULL;

			// 3. reset the Txlength
			pSPIHandle->TxLength = 0;

			// 4. mark the SPI state as ready
			pSPIHandle->TxState = SPI_STATE_READY;

			// 5. execute application event call back function
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EV_TX_CMPLT);
		}

}



/******************************************************************************
 * @fn					- SPIx RXNE Interrupt Handler
 *
 * @brief				- Handle the data when RXNE interrupt is occured
 *
 * @params				- pointer to the Handle structure
 *
 * @return				- void
 *
 * @note				-

 */
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	// 1. Check for the data frame format whether it is 8bit or 16bit

		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// if DFF is 16bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;		// reading data from the data register
			pSPIHandle->RxLength--;												// decrementing the length two times
			pSPIHandle->RxLength--;
			(uint16_t*)pSPIHandle->pRxBuffer++;									// Incrementing RxBuffer
		}
		else
		{
			// if DFF is 8bit
			*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;					// reading data to the data register
			pSPIHandle->RxLength--;									// decrementing the length
			pSPIHandle->pRxBuffer++;								// Incrementing RxBuffer

		}

		// Once everything is completed check whether the length is 0 or not
		// if so, then end the reception
		if(!pSPIHandle->RxLength)
		{
			// 1. reset the RXNEIE
			pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

			// 2. reset the RxBuffer
			pSPIHandle->pRxBuffer = NULL;

			// 3. reset the Rxlength
			pSPIHandle->RxLength = 0;

			// 4. mark the SPI state as ready
			pSPIHandle->RxState = SPI_STATE_READY;

			// 5. execute application event call back function
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EV_RX_CMPLT);
		}
}


/******************************************************************************
 * @fn					- SPIx OVR Interrupt Handler
 *
 * @brief				- Handle the data when OVR interrupt is occured
 *
 * @params				- pointer to the Handle structure
 *
 * @return				- void
 *
 * @note				-


 */
static void SPI_OVR_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
		//read the status and Data registers
		uint8_t temp;
		if(pSPIHandle->TxState != SPI_STATE_BUSY_IN_TX)
		{
			temp = pSPIHandle->pSPIx->SR;
			temp = pSPIHandle->pSPIx->DR;
			(void)temp;
		}
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EV_OVR_ERR);
}



/******************************************************************************
 * @fn					- SPIx Application Event Callback
 *
 * @brief				- Callback function for the Interrupt handler
 *
 * @params				- pointer to the Handle structure
 * @params				- application Event
 *
 * @return				- void
 *
 * @note				- attributed as weak


 */

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent)
{

}
