#include "main.h"
#include "stdio.h"

int _write(int32_t file, uint8_t *ptr, int32_t len) {
	for (int i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

/* 	CHECKLIST
-Turn on the HSE (High Speed External)
-Activate prefetch buffer
-Change wait states/Latency
-Configure RCC and PLL settings while the PLL is off
-Turn on PLL
-Change clock source
*/
void CLOCK_INIT()
{
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)); //wait for HSE to stabilize
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	while(!(FLASH->ACR & FLASH_ACR_PRFTBS));
	FLASH->ACR &= ~(FLASH_ACR_LATENCY_0 | FLASH_ACR_LATENCY_2) ;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	/* PLL CONFIG */
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC);
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_3;
	RCC->CFGR |= RCC_CFGR_PLLMUL_0 | RCC_CFGR_PLLMUL_1 | RCC_CFGR_PLLMUL_2;
	RCC->CFGR |= RCC_CFGR_PLLSRC;
	RCC->CFGR |= RCC_CFGR_PPRE1_2;

	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY))

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));

}
void TIM2_PWM_INIT() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR |= 3 << GPIO_OSPEEDER_OSPEEDR15_Pos;
	GPIOA->MODER |= GPIO_MODER_MODER15_1;   //PA15 alternate function mode
	GPIOA->AFR[1] |= 1 << 28;  		//AF1 (TIM2 CH1 output)
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	//enable clock for TIM2
	TIM2->CCER |= TIM_CCER_CC1E; 		//enable channel 1
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; //pwm mode 1
	TIM2->PSC = 72 - 1;		//prescaler
	TIM2->ARR = 40 - 1;
	TIM2->CCR1 = 20;
	TIM2->EGR |= TIM_EGR_UG; //update registers
	TIM2->CR1 |= TIM_CR1_CEN; // start timer
}

void TIM1_IC_INIT(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;		//ENABLE GPIOA CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;	//ENABLE TIMER CLOCK
	RCC->CFGR3 |= RCC_CFGR3_TIM1SW;
	GPIOA->MODER |= GPIO_MODER_MODER8_1; //SET GPIO TO ALETERNATE FUNCTION

	GPIOA->AFR[1] |= 6 << GPIO_AFRH_AFRH0_Pos; //AF6 FOR PA8 , TIM1_CH1

	//TIM1->SMCR |= 5 << TIM_SMCR_TS_Pos | TIM_SMCR_SMS_2;

	TIM1->CCMR1 |= 1 << TIM_CCMR1_CC1S_Pos;
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);	//RISING EDGE POLARITY
	TIM1->CCMR1 |= 3 << TIM_CCMR1_IC1PSC_Pos;

	TIM1->CCMR1 |= 2 << TIM_CCMR1_CC2S_Pos;		//TI1 IS INPUT TO CHANNEL 2
	TIM1->CCMR1 |= 3 << TIM_CCMR1_IC2PSC_Pos;	//INPUT PRESCALER SET TO 8
	TIM1->CCER |= TIM_CCER_CC2P;				//FALLING EDGE POLARITY

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;				//ENABLE CAPTURE
	TIM1->CR1 |= TIM_CR1_CEN; // START TIMER
}

int main(void)
{
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
	uint32_t time_period = 0, on_time = 0, temp = 0;
	int falling_1 = 0, falling_2 = 0, rising_1 = 0, rising_2 = 0;
	int NOT_WARPED = 1;
	float freq = 0, pulse_width_ns = 0;
	printf("TEST");

	CLOCK_INIT();
	TIM2_PWM_INIT();
	TIM1_IC_INIT();

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;				//ENABLE CAPTURE
	TIM1->CR1 |= TIM_CR1_CEN; // START TIMER

	SystemCoreClockUpdate();

	while (1) {

		while (NOT_WARPED) {

			TIM1->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF); //update registers
			while (!(TIM1->SR & TIM_SR_CC1IF))
				;
			rising_1 = TIM1->CCR1;

			while (!(TIM1->SR & TIM_SR_CC2IF))
				;
			falling_1 = TIM1->CCR2;

			while (!(TIM1->SR & TIM_SR_CC1IF))
				;
			rising_2 = TIM1->CCR1;

			while (!(TIM1->SR & TIM_SR_CC2IF))
				;
			falling_2 = TIM1->CCR2;

			if ((rising_2 > rising_1) && (falling_2 > falling_1))
				NOT_WARPED = 0;
		}
		time_period = rising_2 - rising_1;

		if (falling_1 > rising_1)
			on_time = falling_1 - rising_1;
		else
			on_time = rising_1 - falling_1;

		freq = 8*144000000/time_period;
		pulse_width_ns = on_time/0.144;
		//printf("Frequency=%f , ON-time (ns)=%f,%f,%u\n", freq , pulse_width_ns , time_period, on_time);

		NOT_WARPED = 1;
	}
}
