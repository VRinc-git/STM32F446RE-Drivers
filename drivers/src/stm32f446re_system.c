/*
 * stm32f446re_system.c
 *
 *  Created on: 05-Oct-2020
 *      Author: vishn
 */

#include "stm32f446re_system.h"

/********************************* NVIC API's **********************************/


/******************************************************************************
 * @fn					- NVIC Enable IRQ
 *
 * @brief				- Enable the Interrupt for the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- void
 *
 * @note				-


 */
void NVIC_EnableIRQ(IRQn_Type IRQNumber)
{
	NVIC->ISER[IRQNumber >> 5] |= (1 << (IRQNumber & 0x1F));
}




/******************************************************************************
 * @fn					- NVIC Disable IRQ
 *
 * @brief				- Disable the Interrupt for the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- void
 *
 * @note				-


 */
void NVIC_DisableIRQ(IRQn_Type IRQNumber)
{
	NVIC->ICER[IRQNumber >> 5] |= (1 << (IRQNumber & 0x1F));
}



/******************************************************************************
 * @fn					- NVIC Get pending IRQ
 *
 * @brief				- return the pending status of the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- pending state
 *
 * @note				-


 */
uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQNumber)
{
	return (uint32_t) (((NVIC->ISPR[IRQNumber >> 5]) & (1 << (IRQNumber & 0x1F))) !=0) ? 1 : 0;
}



/******************************************************************************
 * @fn					- NVIC set pending IRQ
 *
 * @brief				- set the pending status for the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- void
 *
 * @note				-


 */
void NVIC_SetPendingIRQ(IRQn_Type IRQNumber)
{
	NVIC->ISPR[IRQNumber >> 5] |= (1 << (IRQNumber & 0x1F));
}



/******************************************************************************
 * @fn					- NVIC clear pending IRQ
 *
 * @brief				- clear the pending status for the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- void
 *
 * @note				-


 */
void NVIC_ClearPendingIRQ(IRQn_Type IRQNumber)
{
	NVIC->ICPR[IRQNumber >> 5] |= (1 << (IRQNumber & 0x1F));
}



/******************************************************************************
 * @fn					- NVIC get Active IRQ
 *
 * @brief				- return the active status for the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- Active status
 *
 * @note				-


 */
uint32_t NVIC_GetActive(IRQn_Type IRQNumber)
{
	return (uint32_t) (((NVIC->IABR[IRQNumber >> 5]) & (1 << (IRQNumber & 0x1F))) != 0) ? 1 : 0;
}



/******************************************************************************
 * @fn					- NVIC set priority
 *
 * @brief				- set the priority for the given peripherals
 *
 * @params				- IRQ Number
 * @params				- Priority of the interrupt
 *
 * @return				- void
 *
 * @note				-


 */
void NVIC_SetPriority(IRQn_Type IRQNumber, uint32_t priority)
{
	if(IRQNumber < 0) {
	    SCB->SHP[(((uint32_t)IRQNumber) & 0xF) - 4] = ((priority << (8 - NVIC_PRIO_BITS)) & 0xFF);
	}
	else {
		NVIC->IP[(uint32_t)IRQNumber] |= ((priority <<  (8 - NVIC_PRIO_BITS)) & 0xFF);
	}
}



/******************************************************************************
 * @fn					- NVIC Get priority
 *
 * @brief				- return the priority of the given peripherals
 *
 * @params				- IRQ Number
 *
 * @return				- priority number
 *
 * @note				-


 */
uint32_t NVIC_GetPriority(IRQn_Type IRQNumber)
{
	if(IRQNumber < 0) {
	    return(((uint32_t)SCB->SHP[(((uint32_t)IRQNumber) & 0xF) - 4] >> (8 - NVIC_PRIO_BITS)));
	}
	else {
		return (uint32_t)((NVIC->IP[(uint32_t)IRQNumber] >> ((8 - NVIC_PRIO_BITS))));
	}
}


/************************************ Systick API's *********************************/

/******************************************************************************
 * @fn					- SysTick Config
 *
 * @brief				- Configure the SysTick
 *
 * @params				- number of ticks
 *
 * @return				- return status
 *
 * @note				-


 */
uint32_t SysTick_Config(uint32_t ticks)
{
	if((ticks - 1) > 0xFFFFFF ){
		return 1;
	}

	//Disable systick
	SysTick->CTRL &= ~(1 << 0);

	//Set reload register
	SysTick->LOAD = ticks - 1;

	//set interrupt priority for systick (Highest priority)
	NVIC_SetPriority(SysTick_IRQn, (1 << NVIC_PRIO_BITS) - 1);

	//Reset the systick counter value
	SysTick->VAL = 0;

	//select the processor clock
	SysTick->CTRL |= (1 << 2);

	//Enable the systick interrupt
	SysTick->CTRL |= (1 << 1);

	//Enable systick
	SysTick->CTRL |= (1 << 0);

	return 0;

}



/******************************************************************************
 * @fn					- SysTick Get Current Value
 *
 * @brief				- return the Current value of the System timer
 *
 * @params				-
 *
 * @return				- Current value of the systick
 *
 * @note				-


 */
uint32_t SysTick_GetCurrentValue()
{
	return (uint32_t) ((SysTick->VAL) & 0xFFFFFF);
}
