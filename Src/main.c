

#include "stm32f446re_gpio.h"

GPIO_Handle_t led, butn;

#if 0
void led_init()
{
	led.pGPIOx = GPIOA;
	led.Init.GPIO_PinMode = GPIO_MODE_Output;
	led.Init.GPIO_PinOutputType = GPIO_OP_TYPE_PushPull;
	led.Init.GPIO_PinOutputSpeed = GPIO_OSPEED_Low;
	led.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	led.Init.GPIO_PinNumber = GPIO_Pin5;
	GPIO_Init(&led);
}
void Butn_init()
{
	butn.pGPIOx = GPIOC;
	butn.Init.GPIO_PinMode = GPIO_MODE_Input;
	butn.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	butn.Init.GPIO_PinNumber = GPIO_Pin13;
	GPIO_Init(&butn);
}

int main(void)
{
	led_init();
	Butn_init();

	while(1)
	{
		while(GPIO_ReadPin(GPIOC, GPIO_Pin13));
		GPIO_TogglePin(GPIOA, GPIO_Pin5);

	}
}
#endif


#if	0

void led_init()
{
	led.pGPIOx = GPIOA;
	led.Init.GPIO_PinMode = GPIO_MODE_Output;
	led.Init.GPIO_PinOutputType = GPIO_OP_TYPE_PushPull;
	led.Init.GPIO_PinOutputSpeed = GPIO_OSPEED_Low;
	led.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	led.Init.GPIO_PinNumber = GPIO_Pin5;
	GPIO_Init(&led);
}

void Butn_init()
{
	butn.pGPIOx = GPIOC;
	butn.Init.GPIO_PinMode = GPIO_MODE_IT_FE;
	butn.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	butn.Init.GPIO_PinNumber = GPIO_Pin13;
	GPIO_Init(&butn);

	//IRQ
	NVIC_SetPriority(EXTI15_10_IRQn, 1);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}


int main(void)
{
	led_init();
	Butn_init();

	while(1);

}

void EXTI15_10_IRQHandler(void){

	GPIO_IRQHandle(GPIO_Pin13);
	GPIO_TogglePin(GPIOA, GPIO_Pin5);

}
#endif



#if 1
__vo uint32_t TimeDelay = 0;

void delay(uint32_t time);


void led_init()
{
	led.pGPIOx = GPIOA;
	led.Init.GPIO_PinMode = GPIO_MODE_Output;
	led.Init.GPIO_PinOutputType = GPIO_OP_TYPE_PushPull;
	led.Init.GPIO_PinOutputSpeed = GPIO_OSPEED_Low;
	led.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	led.Init.GPIO_PinNumber = GPIO_Pin5;
	GPIO_Init(&led);
}
void Butn_init()
{
	butn.pGPIOx = GPIOC;
	butn.Init.GPIO_PinMode = GPIO_MODE_Input;
	butn.Init.GPIO_PinPullUpDown = GPIO_PIN_NoPuPD;
	butn.Init.GPIO_PinNumber = GPIO_Pin13;
	GPIO_Init(&butn);
}

int main(void)
{
	SysTick_Config(16000);
	led_init();
	Butn_init();

	while(1)
	{
		delay(500);
		GPIO_TogglePin(GPIOA, GPIO_Pin5);

	}
}


void delay(uint32_t time){

	TimeDelay = time;
	while(TimeDelay != 0);
}

void SysTick_Handler(void){
	if(TimeDelay > 0){
		TimeDelay--;
	}
}
#endif
