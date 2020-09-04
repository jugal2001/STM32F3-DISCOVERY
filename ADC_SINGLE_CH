#include "stm32f303xc.h"

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
void adc_init(void)
{
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; 		//enable clock 
	ADC1->CR &= ~(ADC_CR_ADVREGEN_1 | ADC_CR_ADVREGEN_0);
	ADC1->CR |= ADC_CR_ADVREGEN_0;//voltage regulator enable sequence
	delay_ms(10);
	ADC1_2_COMMON->CCR|=1<<16;
	ADC1->CR &= ~ADC_CR_ADCALDIF;//single ended mode calibration
	ADC1->CR |= ADC_CR_ADCAL; //start calibration
	while((ADC1->CR)& ADC_CR_ADCAL); //wait till calibration complete
	delay_ms(10);
	ADC1->CR |= ADC_CR_ADEN;//enable ADC
	while(!( (ADC1->ISR) & ADC_ISR_ADRDY )); //wait till ADC is ready
	ADC1->SQR1 |= ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_0;//channel 3 in sequence 1
	ADC1->SMPR1 |= ADC_SMPR1_SMP3_2 | ADC_SMPR1_SMP3_1; 		 		//set sampling rate
	ADC1->CFGR |= ADC_CFGR_CONT;  			//set to continuous conversion mode
	ADC1->CR |= ADC_CR_ADSTART;
}
void pwm_init(void)
{
	RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN;	//enable clock for TIM2
	TIM2 ->CCER |= TIM_CCER_CC1E; 		//enable channel 1
	TIM2 ->CR1 |= TIM_CR1_ARPE;
 TIM2 ->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; //pwm mode 1
	TIM2 ->PSC = 1;		//prescaler
	TIM2 ->ARR = 4095;
	TIM2 ->CCR1 = 0;
	TIM2 ->EGR = TIM_EGR_UG; //update registers
	TIM2 ->CR1 = TIM_CR1_CEN; // start timer
}

int main()
{
		//***ADC PA2 , LED ON PA15***//
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //Enable clock for port A
	GPIOA->MODER |= GPIO_MODER_MODER15_1; //PA15 alternate function mode
	GPIOA->AFR[1] |= 1<<28; 	//AF1 (TIM2 CH1 output)
	 
	//PA2  (connected to ADC1 ch3)
	 GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER2_0;//analog mode

	//set up systick (1ms per interrupt)
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	adc_init();
	pwm_init();

while(1)
{

	//if(ADC1->ISR & ADC_ISR_EOC) // Test EOC flag
		TIM2->CCR1 = ADC1->DR;
}

}

