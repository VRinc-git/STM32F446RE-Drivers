/*
 * stm32f446re_gpio.h
 *
 *  Created on: 28-Sep-2020
 *      Author: vishn
 */
#include "stm32f446re_gpio.h"

/*************************** GPIOx Driver support APIs ******************************/


/******************************************************************************
 * @fn					- GPIOx Peripheral clock Control
 *
 * @brief				- Enable or Disable the clock for GPIOx peripherals
 *
 * @params				- pointer to the register definition
 * @params				- ENABLE or DISABLE
 *
 * @return				- void
 *
 * @note				-


 */

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}
}


/******************************************************************************
 * @fn					- GPIOx Peripheral Initialization
 *
 * @brief				- Initialize the GPIOx peripherals
 *
 * @params				- pointer to the Handle
 *
 * @return				- void
 *
 * @note				-


 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//Enable the peripheral clock
	GPIO_PCLKControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp_reg = 0;

	//Configuring the mode
	if(pGPIOHandle->Init.GPIO_PinMode <= GPIO_MODE_Analog)
	{
		temp_reg &= ~(pGPIOHandle->Init.GPIO_PinMode << (2 * pGPIOHandle->Init.GPIO_PinNumber));
		temp_reg |= (pGPIOHandle->Init.GPIO_PinMode << (2 * pGPIOHandle->Init.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp_reg;
	}
	else
	{
		if(pGPIOHandle->Init.GPIO_PinMode == GPIO_MODE_IT_FE)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->Init.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->Init.GPIO_PinNumber);
		}
		else if(pGPIOHandle->Init.GPIO_PinMode == GPIO_MODE_IT_RE)
		{
			EXTI->FTSR &= ~(1 << pGPIOHandle->Init.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->Init.GPIO_PinNumber);
		}
		else if(pGPIOHandle->Init.GPIO_PinMode == GPIO_MODE_IT_FE_RE)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->Init.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->Init.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in the SYSCFG_EXTICR
		uint8_t row = 0;
		uint8_t column = 0;
		row = pGPIOHandle->Init.GPIO_PinNumber / 4;
		column = pGPIOHandle->Init.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_PORTCODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[row] |= (portCode << (4 * column));

		// enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->Init.GPIO_PinNumber);


	}

	//Configure the output type
	temp_reg = 0;
	temp_reg |= (pGPIOHandle->Init.GPIO_PinOutputType << pGPIOHandle->Init.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp_reg;

	//configure the output speed
	temp_reg = 0;
	temp_reg |= (pGPIOHandle->Init.GPIO_PinOutputSpeed <<(2 * pGPIOHandle->Init.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp_reg;

	//configure the Pull up down register
	temp_reg = 0;
	temp_reg |= (pGPIOHandle->Init.GPIO_PinPullUpDown << (2 * pGPIOHandle->Init.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp_reg;

	//Configure alternate function
	uint8_t row = 0;
	uint8_t column = 0;
	row = pGPIOHandle->Init.GPIO_PinNumber / 8;
	column = pGPIOHandle->Init.GPIO_PinNumber % 8;
	pGPIOHandle->pGPIOx->AFR[row] |= pGPIOHandle->Init.GPIO_PinAltFn << (4 * column);

}


/******************************************************************************
 * @fn					- GPIOx Set Pin
 *
 * @brief				- Set pin for GPIOx peripherals
 *
 * @params				- pointer to the Register definition
 * @params				- Pin number to be set
 * @params				- SET or RESET
 *
 * @return				- void
 *
 * @note				-


 */
void GPIO_SetPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	pGPIOx->ODR |= (Value << PinNumber);
}

/******************************************************************************
 * @fn					- GPIOx Set Port
 *
 * @brief				- Set Port for GPIOx peripherals
 *
 * @params				- pointer to the Register definition
 * @params				- SET or RESET
 *
 * @return				- void
 *
 * @note				-


 */
void GPIO_SetPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR |= Value;
}

/******************************************************************************
 * @fn					- GPIOx Read Pin
 *
 * @brief				- Read pin for GPIOx peripherals
 *
 * @params				- pointer to the Register definition
 * @params				- Pin number to be Read
 *
 * @return				- void
 *
 * @note				-


 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value = 0;
	Value |= (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return Value;
}

/******************************************************************************
 * @fn					- GPIOx Read Port
 *
 * @brief				- Read port for GPIOx peripherals
 *
 * @params				- pointer to the Register definition
 *
 * @return				- void
 *
 * @note				-


 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value = 0;
	Value |= pGPIOx->IDR;
	return Value;
}

/******************************************************************************
 * @fn					- GPIOx Toggle Pin
 *
 * @brief				- Toggle pin for GPIOx peripherals
 *
 * @params				- pointer to the Register definition
 * @params				- Pin number to be Toggled
 *
 * @return				- void
 *
 * @note				-


 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * Interrupt
 */
/******************************************************************************
 * @fn					- GPIOx IRQ Handler
 *
 * @brief				- Handle the IRQ of the GPIOx peripherals
 *
 * @params				- pin number
 *
 * @return				- void
 *
 * @note				-


 */
void GPIO_IRQHandle(uint8_t PinNumber)
{
	//Pending the EXTI PR register
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
