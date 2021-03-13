/*
 * stm32f446re_gpio.h
 *
 *  Created on: 28-Sep-2020
 *      Author: vishn
 */

#ifndef INC_STM32F446RE_GPIO_H_
#define INC_STM32F446RE_GPIO_H_

#include "stm32f446re.h"

typedef struct
{
	uint32_t	GPIO_PinNumber;
	uint32_t	GPIO_PinMode;
	uint32_t	GPIO_PinOutputType;
	uint32_t	GPIO_PinOutputSpeed;
	uint32_t	GPIO_PinPullUpDown;
	uint32_t	GPIO_PinAltFn;


}GPIO_Config_t;


typedef struct
{
	GPIO_RegDef_t  *pGPIOx;
	GPIO_Config_t	Init;


}GPIO_Handle_t;



/**************Macros**************/
//@GPIO_PinNumber
#define GPIO_Pin0		0
#define GPIO_Pin1		1
#define GPIO_Pin2		2
#define GPIO_Pin3		3
#define GPIO_Pin4		4
#define GPIO_Pin5		5
#define GPIO_Pin6		6
#define GPIO_Pin7		7
#define GPIO_Pin8		8
#define GPIO_Pin9		9
#define GPIO_Pin10		10
#define GPIO_Pin11		11
#define GPIO_Pin12		12
#define GPIO_Pin13		13
#define GPIO_Pin14		14
#define GPIO_Pin15		15

//@GPIO_PinMode;
#define GPIO_MODE_Input		0
#define GPIO_MODE_Output	1
#define GPIO_MODE_AltFn		2
#define GPIO_MODE_Analog	3
#define GPIO_MODE_IT_FE		4
#define GPIO_MODE_IT_RE		5
#define GPIO_MODE_IT_FE_RE	6


//@GPIO_PinOutputType
#define GPIO_OP_TYPE_PushPull	0
#define GPIO_OP_TYPE_OpenDrain	1

//@GPIO_PinOutputSpeed
#define GPIO_OSPEED_Low			0
#define GPIO_OSPEED_Medium		1
#define GPIO_OSPEED_Fast		2
#define GPIO_OSPEED_High		3

//@GPIO_PinPullUpDown
#define GPIO_PIN_NoPuPD			0
#define GPIO_PIN_PullUp			1
#define GPIO_PIN_PullDown		2

//@GPIO_PinAltFn
#define GPIO_ALTFN_AF0			0
#define GPIO_ALTFN_AF1			1
#define GPIO_ALTFN_AF2			2
#define GPIO_ALTFN_AF3			3
#define GPIO_ALTFN_AF4			4
#define GPIO_ALTFN_AF5			5
#define GPIO_ALTFN_AF6			6
#define GPIO_ALTFN_AF7			7
#define GPIO_ALTFN_AF8			8
#define GPIO_ALTFN_AF9			9
#define GPIO_ALTFN_AF10			10
#define GPIO_ALTFN_AF11			11
#define GPIO_ALTFN_AF12			12
#define GPIO_ALTFN_AF13			13
#define GPIO_ALTFN_AF14			14
#define GPIO_ALTFN_AF15			15


/****************** GPIOx Driver support APIs ******************/

//GPIOx Peripheral clock Control
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * read and write
 */
void GPIO_SetPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_SetPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Interrupt configuration
 */
void GPIO_IRQHandle(uint8_t PinNumber);


#endif /* INC_STM32F446RE_GPIO_H_ */
