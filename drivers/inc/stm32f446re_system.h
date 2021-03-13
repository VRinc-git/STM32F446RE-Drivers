/*
 * stm32f446re_system.h
 *
 *  Created on: 05-Oct-2020
 *      Author: vishn
 */

#ifndef INC_STM32F446RE_SYSTEM_H_
#define INC_STM32F446RE_SYSTEM_H_


#include "stdint.h"
#include "stm32f446re.h"


/**************************************** Cortex M4F *********************************************/


/*
 * NVIC base address
 */
#define NVIC_BASEADDR 		0xE000E100U

typedef struct
{
	__IO uint32_t ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
	uint32_t RESERVED0[24];
	__IO uint32_t ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
	uint32_t RSERVED1[24];
	__IO uint32_t ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
	uint32_t RESERVED2[24];
	__IO uint32_t ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
	uint32_t RESERVED3[24];
	__IO uint32_t IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
	uint32_t RESERVED4[56];
	__IO uint8_t  IP[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
	uint32_t RESERVED5[644];
	__IO uint32_t STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
}  NVIC_Type;



#define NVIC                ((NVIC_Type*) NVIC_BASEADDR)

#define NVIC_PRIO_BITS		4


/*
 * System Control block
 */

#define SCB_BASEADDR 		0xE000ED00U

typedef struct
{
	__I  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
	__IO uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
	__IO uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
	__IO uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
	__IO uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
	__IO uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
	__IO uint8_t  SHP[12];                 /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	__IO uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
	__IO uint32_t CFSR;                    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
	__IO uint32_t HFSR;                    /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
	__IO uint32_t DFSR;                    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
	__IO uint32_t MMFAR;                   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
	__IO uint32_t BFAR;                    /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
	__IO uint32_t AFSR;                    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
	__I  uint32_t PFR[2];                  /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
	__I  uint32_t DFR;                     /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
	__I  uint32_t ADR;                     /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
	__I  uint32_t MMFR[4];                 /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
	__I  uint32_t ISAR[5];                 /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
	uint32_t RESERVED0[5];
	__IO uint32_t CPACR;                   /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
} SCB_Type;

#define SCB			((SCB_Type*) SCB_BASEADDR)



/****************************************** System timer (SysTick) *******************************************/

#define SysTick_BASE        0xE000E010U

typedef struct
{
	__IO uint32_t CTRL;                    /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
	__IO uint32_t LOAD;                    /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register       */
	__IO uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register      */
	__I  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register        */
} SysTick_Type;


#define SysTick             ((SysTick_Type*) SysTick_BASE  )


/**************************** NVIC API's ****************************/

void NVIC_EnableIRQ(IRQn_Type IRQNumbers);
void NVIC_DisableIRQ(IRQn_Type IRQNumbers);

uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQNumber);
void NVIC_SetPendingIRQ(IRQn_Type IRQNumber);
void NVIC_ClearPendingIRQ(IRQn_Type IRQNumber);

uint32_t NVIC_GetActive(IRQn_Type IRQNumber);

void NVIC_SetPriority(IRQn_Type IRQNumber, uint32_t priority);
uint32_t NVIC_GetPriority(IRQn_Type IRQNumber);


/**************************** SYSTICK API's ****************************/
uint32_t SysTick_Config(uint32_t ticks);
uint32_t SysTick_GetCurrentValue();




/*************************** Macros ****************************/



#endif /* INC_STM32F446RE_SYSTEM_H_ */
