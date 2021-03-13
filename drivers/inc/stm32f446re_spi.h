/*
 * stm32f446re_spi.h
 *
 *  Created on: Feb 28, 2021
 *      Author: tamor
 */

#ifndef INC_STM32F446RE_SPI_H_
#define INC_STM32F446RE_SPI_H_

#include "stm32f446re.h"


typedef struct
{
	uint32_t SPI_DeviceMode;
	uint32_t SPI_BusConfig;
	uint32_t SPI_DataFrameFormat;
	uint32_t SPI_ClockPhase;
	uint32_t SPI_ClockPolarity;
	uint32_t SPI_SlaveManagement;
	uint32_t SPI_BaudRate;

}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t Init;
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
	uint32_t TxLength;
	uint32_t RxLength;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;


//SPI application state
#define SPI_STATE_READY				0
#define SPI_STATE_BUSY_IN_TX		1
#define SPI_STATE_BUSY_IN_RX		2

//SPI application events
#define SPI_EV_TX_CMPLT		0
#define SPI_EV_RX_CMPLT		1
#define SPI_EV_OVR_ERR		2


/*********************************** MACROS ****************************************/

//@SPI_DeviceMode
#define SPI_MODE_MASTER			1
#define SPI_MODE_SLAVE			0

//@SPI_BusConfig
#define SPI_BUS_FD					1
#define SPI_BUS_HD					2
#define SPI_BUS_SIMPLEX_RX_ONLY		3

//@SPI_DataFrameFormat
#define SPI_DATA_FRAME_8BIT			0
#define SPI_DATA_FRAME_16BIT		1

//@SPI_ClockPhase
#define SPI_CLOCK_PHASE_LOW			0
#define SPI_CLOCK_PHASE_HIGH		1

//@SPI_ClockPolarity
#define SPI_CLOCK_POLARITY_LOW		0
#define SPI_CLOCK_POLARITY_HIGH		1


//@SPI_SlaveManagement
#define SPI_SLAVE_MANAGEMENT_HW		0
#define SPI_SLAVE_MANAGEMENT_SW		1

//@SPI_BaudRate
#define SPI_BAUD_RATE_DIV2			0
#define SPI_BAUD_RATE_DIV4			1
#define SPI_BAUD_RATE_DIV8			2
#define SPI_BAUD_RATE_DIV16			3
#define SPI_BAUD_RATE_DIV32			4
#define SPI_BAUD_RATE_DIV64			5
#define SPI_BAUD_RATE_DIV128		6
#define SPI_BAUD_RATE_DIV256		7


/*************************** SPIx Driver support APIs  ******************************/
//SPIx Peripheral clock Control
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

/*
 * Interrupt Handle
 */
void SPI_IRQHandle(SPI_Handle_t *pSPIHandle);

/*
 * Other functions
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent);

/*************************** SPI PIN MACROS **************************/
//SPI Control Register 1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

//SPI Control Register 2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//SPI Status Register
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


//SPI Flags
#define SPI_FLAG_RXNE		(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE		(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE		(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR		(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR		(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF		(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR		(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY		(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE		(1 << SPI_SR_FRE)

#endif /* INC_STM32F446RE_SPI_H_ */
