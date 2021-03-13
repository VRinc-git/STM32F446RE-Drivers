/*
 * stm32f446re.h
 *
 *  Created on: 26-Sep-2020
 *      Author: vishn
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_



#include "stdint.h"
#include "stdlib.h"

#define __vo	volatile
#define __IO	__vo
#define	__I		volatile const

/* 8, 16 & 32 Bit Register Access Macros */
#define HWREG8(x)	(*((__vo uint8_t*)(x)))
#define HWREG16(x)	(*((__vo uint16_t*)(x)))
#define HWREG32(x)	(*((__vo uint32_t*)(x)))

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;



#include "stm32f446re_system.h"


/*************************************** STM32f446RE *******************************************/

/*
 * Memory Base addresses
 */
#define FLASH_BASEADDR		()0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM					0x1FFF0000U
#define SRAM				SRAM1_BASEADDR

/*
 * Bus interface Base Addresses
 */
#define APB1_BASEADDR		0x40000000U
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U
#define AHB3_BASEADDR		0xA0001000U


/*
 * Register Boundary addresses
 */
//AHB1
#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)
#define CRC_BASEADDR				(AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800)
#define FLASH_INTERFACE_BASEADDR	(AHB1_BASEADDR + 0x3C00)
#define BKPSRAM_BASEADDR			(AHB1_BASEADDR + 0x4000)
#define DMA1_BASEADDR				(AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR				(AHB1_BASEADDR + 0x6400)
#define USB_OTG_HS_BASEADDR			(AHB1_BASEADDR + 0x20000)

//APB1
#define TIM2_BASEADDR				(APB1_BASEADDR + 0x0000)
#define TIM3_BASEADDR				(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR				(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR				(APB1_BASEADDR + 0x0C00)
#define TIM6_BASEADDR				(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR				(APB1_BASEADDR + 0x1400)
#define TIM12_BASEADDR				(APB1_BASEADDR + 0x1800)
#define TIM13_BASEADDR				(APB1_BASEADDR + 0x1C00)
#define TIM14_BASEADDR				(APB1_BASEADDR + 0x2000)
#define RTC_BKP_BASEADDR			(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR				(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR				(APB1_BASEADDR + 0x3000)
#define SPI2_I2S2_BASEADDR			(APB1_BASEADDR + 0x3800)
#define SPI3_I2S3_BASEADDR			(APB1_BASEADDR + 0x3C00)
#define SPDIF_RX_BASEADDR			(APB1_BASEADDR + 0x4000)
#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000)
#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)
#define CAN1_BASEADDR				(APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR				(APB1_BASEADDR + 0x6800)
#define HDMI_CEC_BASEADDR			(APB1_BASEADDR + 0x6C00)
#define PWR_BASEADDR				(APB1_BASEADDR + 0x7000)
#define DAC_BASEADDR				(APB1_BASEADDR + 0x7400)


//APB2
#define TIM1_BASEADDR				(APB2_BASEADDR + 0x0000)
#define TIM8_BASEADDR				(APB2_BASEADDR + 0x0400)
#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400)
#define ADC_BASEADDR				(APB2_BASEADDR + 0x2000)
#define SDMMC_BASEADDR				(APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)
#define TIM9_BASEADDR				(APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR				(APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR				(APB2_BASEADDR + 0x4800)
#define SAI1_BASEADDR				(APB2_BASEADDR + 0x5800)
#define SAI2_BASEADDR				(APB2_BASEADDR + 0x5C00)



/*********************************** Peripheral Register definitions ****************************************/


/*
 * Structure definition for RCC registers
 */

typedef struct
{
	__IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	__IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	__IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	__IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	__IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	__IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	__IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	__IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	__IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	__IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	__IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	__IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	__IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	__IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	__IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	__IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	__IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	__IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	__IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	__IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	__IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
	__IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
	__IO uint32_t CKGATENR;      /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
	__IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */

}RCC_RegDef_t;



/*
 * Structure definition for GPIOx registers
 */

typedef struct
{
	__IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	__IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	__IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	__IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	__IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	__IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	__IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	__IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	__IO uint32_t AFR[2];   /*!< <AFR[0] - Alternate function Low register					Address offset = 0x20
								 AFR[1] - Alternate function High register					Address offset = 0x24  */
}GPIO_RegDef_t;


/*
 * Structure definition for EXTI registers
 */

typedef struct
{
	__IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
	__IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
	__IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
	__IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
	__IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
	__IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
}EXTI_RegDef_t;





/*
 * Structure definition for SYSCFG registers
 */

typedef struct
{
	__IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
	__IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
	__IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
	__IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
	uint32_t      RESERVED1[2]; /*!< Reserved, 0x24-0x28                                                          */
	__IO uint32_t CFGR;         /*!< SYSCFG Configuration register,                     Address offset: 0x2C      */
}SYSCFG_RegDef_t;


/*
 * Structure definition for SPI
 */

typedef struct
{
	__IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
	__IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
	__IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
	__IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
	__IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
	__IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
	__IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
	__IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
	__IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
}SPI_RegDef_t;



/*
 *
 * Peripheral definition macros
 *
 */
//GPIOx

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)


//RCC
#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

//SYSCFG
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

//EXTI
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

//SPI
#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define	SPI3				((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)


/************************************ Peripheral Clock enable macros  *****************************************/

//GPIOx
#define GPIOA_PCLK_EN()			RCC->AHB1ENR |= (1<<0)
#define GPIOB_PCLK_EN()			RCC->AHB1ENR |= (1<<1)
#define GPIOC_PCLK_EN()			RCC->AHB1ENR |= (1<<2)
#define GPIOD_PCLK_EN()			RCC->AHB1ENR |= (1<<3)
#define GPIOE_PCLK_EN()			RCC->AHB1ENR |= (1<<4)
#define GPIOF_PCLK_EN()			RCC->AHB1ENR |= (1<<5)
#define GPIOG_PCLK_EN()			RCC->AHB1ENR |= (1<<6)
#define GPIOH_PCLK_EN()			RCC->AHB1ENR |= (1<<7)

//SYSCFG
#define SYSCFG_PCLK_EN()		RCC->APB2ENR |= (1<<14)

//SPI
#define SPI1_PCLK_EN()			RCC->APB2ENR |= (1<<12)
#define SPI2_PCLK_EN()			RCC->APB1ENR |= (1<<14)
#define SPI3_PCLK_EN()			RCC->APB1ENR |= (1<<15)


/************************************ Peripheral Clock disable macros  *****************************************/

//GPIOx
#define GPIOA_PCLK_DI()			RCC->AHB1ENR &= ~(1<<0)
#define GPIOB_PCLK_DI()			RCC->AHB1ENR &= ~(1<<1)
#define GPIOC_PCLK_DI()			RCC->AHB1ENR &= ~(1<<2)
#define GPIOD_PCLK_DI()			RCC->AHB1ENR &= ~(1<<3)
#define GPIOE_PCLK_DI()			RCC->AHB1ENR &= ~(1<<4)
#define GPIOF_PCLK_DI()			RCC->AHB1ENR &= ~(1<<5)
#define GPIOG_PCLK_DI()			RCC->AHB1ENR &= ~(1<<6)
#define GPIOH_PCLK_DI()			RCC->AHB1ENR &= ~(1<<7)

//SYSCFG
#define SYSCFG_PCLK_DI()		RCC->APB2ENR &= ~(1<<14)

//SPI
#define SPI1_PCLK_DI()			RCC->APB2ENR &= ~(1<<12)
#define SPI2_PCLK_DI()			RCC->APB1ENR &= ~(1<<14)
#define SPI3_PCLK_DI()			RCC->APB1ENR &= ~(1<<15)


/************************************ Register Reset macros *******************************************/
//GPIOx
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


//SYSCFG
#define SYSCFG_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)


//SPI
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/************************************** PortCode Macros **********************************************/
#define GPIO_PORTCODE(x)		((x == GPIOA) ? 0:\
								 (x == GPIOB) ? 1:\
								 (x == GPIOC) ? 2:\
								 (x == GPIOD) ? 3:\
								 (x == GPIOE) ? 4:\
								 (x == GPIOF) ? 5:\
								 (x == GPIOG) ? 6:\
								 (x == GPIOH) ? 7:0)

/***************************************** Generic Macros **********************************************/

#define SET						1
#define RESET					0
#define ENABLE					SET
#define DISABLE					RESET
#define GPIO_PIN_SET			ENABLE
#define GPIO_PIN_RESET			DISABLE
#define FLAG_SET				SET
#define FLAG_RESET				RESET
#define HIGH					SET
#define LOW						RESET




/***********************************/


#endif /* INC_STM32F446RE_H_ */
