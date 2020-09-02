#include "stm32f3xx.h"


volatile int ticks=0;
void SysTick_Handler(void)
{
	ticks++;
}
void delay_ms(int ms)
{
ticks=0;
while(ticks<ms);
}

int main()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	GPIOE->MODER |= GPIO_MODER_MODER8_0; // PE8 gen purpose output (pushpull by default reset state)
	
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000); // configure system timer to generate interrupt every 1ms
	while(1)
	{
		GPIOE->BSRR |= GPIO_BSRR_BS_8; //set pin high
		delay_ms(1000);
		GPIOE->BSRR |= GPIO_BSRR_BR_8; //set pin low
		delay_ms(1000);
	}
	
}

