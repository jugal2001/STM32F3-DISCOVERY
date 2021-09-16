#include "main.h"
#include "stdlib.h"

CAN_HandleTypeDef hcan;					//struct containing CAN init settings
CAN_FilterTypeDef sFilterConfig;			//struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;				//struct for recieved data frame
uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };	//array for recieved data (8 bytes)

void SystemClock_Config(void);
static void MX_CAN_Init(void);
void Timer_Initialize(void);
void Drive(int DL, int DR, int oct0, int a, int b, int oct1, int p, int q,
		int X, int Y);
int mapp(float k, float l, float h, float L, float H);
void MotorCode(int, int);
void GPIO_Initialize(void);

int main(void) {
	HAL_Init();
	SystemClock_Config();
	Timer_Initialize();
	GPIO_Initialize();
	MX_CAN_Init();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//filter bank consists of 2 32bit values (mask and ID)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//filter set to mask and ID mode
	sFilterConfig.FilterBank = 0;				//filter bank number 0 selected
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//assign filter bank to FIFO 0
	sFilterConfig.FilterIdHigh = 7 << 5;//STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
	sFilterConfig.FilterMaskIdHigh = 7 << 5;//0b111 shifted by 5 for the same reason, first 11 bits are for Identifier											//MSB for ID
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterActivation = ENABLE;		//activate filter

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);	//commits filter settings
	HAL_CAN_Start(&hcan);

	int button = 0;
	int prev_button = 0;
	int k = 0;
	int time = HAL_GetTick();
	uint16_t x = 2000, y = 2000;
	while (1) {
		if ((HAL_GetTick() - time) > 2000) {
			x = 2000;
			y = 2000;
		}
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))//checks if the number of messages in FIFO 0 is non zero
				{
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxMessage, rxData);//stores the data frame in RxMessage struct, stores data in rxData array
			x = rxData[0] | (rxData[1] << 8);
			y = rxData[2] | (rxData[3] << 8);
			time = HAL_GetTick();
		}
		button = (GPIOA->IDR & GPIO_IDR_0);
		if (button && !(prev_button))   //Button Press
				{
			k = ~k;
			prev_button = 1;
		} else
			prev_button = (GPIOA->IDR & GPIO_IDR_0);

		if (k)
			;
		else
			MotorCode(x, y);
	}
}
void GPIO_Initialize() {
	//Enable Clocks:
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   //Enable Clock for Port A
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;   //Enable Clock for Port C
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;   //Enable Clock for Port E
	RCC->AHBENR |= RCC_AHBENR_GPIODEN;   //Enable Clock for Port D
	//Setup PA0:
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);   //Enable as INPUT
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;   //Enable Pull-Down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0);

	//Setup PE15:
	GPIOE->MODER &= ~(GPIO_MODER_MODER15_1);   //Enable as OUTPUT
	GPIOE->MODER |= GPIO_MODER_MODER15_0;
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT_15);   //Output Push-Pull
	GPIOE->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR15);   //High Speed

	//Setup PE13:
	GPIOE->MODER &= ~(GPIO_MODER_MODER13_1);   //Enable as OUTPUT
	GPIOE->MODER |= GPIO_MODER_MODER13_0;
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT_13);   //Output Push-Pull
	GPIOE->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR13);   //High Speed

	//Setup PE9:
	//Enable AF Mode:
	GPIOE->MODER |= GPIO_MODER_MODER9_1;
	GPIOE->MODER &= ~(GPIO_MODER_MODER9_0);
	GPIOE->AFR[1] |= 1 << 5;

	//Setup PE11:
	//Enable AF Mode:
	GPIOE->MODER |= GPIO_MODER_MODER11_1;
	GPIOE->MODER &= ~(GPIO_MODER_MODER11_0);
	GPIOE->AFR[1] |= 1 << 13;

	//Setup PA0:   BUTTON
		GPIOA->MODER &= ~(GPIO_MODER_MODER0);   //Enable as INPUT
		GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;   //Enable Pull-Down
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0);
}
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_CAN_Init(void) {
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 6;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
}



void Timer_Initialize() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   //Enable Timer1
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 & 2 as OUTPUT
	TIM1->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable PreLoad for Channel 1
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable PreLoad for Channel 2

	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); //PWM Mode 1 for Channel 1
	TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); //PWM Mode 1 for Channel 2

	TIM1->PSC = 16;   //freq/1 = 72 Mhz
	TIM1->ARR = 4095;   //16 Bit Value
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;

	TIM1->BDTR |= TIM_BDTR_MOE;   //Main Output Enable

	TIM1->EGR |= TIM_EGR_UG;   //Update Registers
	TIM1->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void Drive(int DL, int DR, int oct0, int a, int b, int oct1, int p, int q,
		int X, int Y) {
	if (DL == 1)
		GPIOE->BSRR |= 1 << 13;   //Turn on LEFT LED
	else
		GPIOE->BRR |= 1 << 13;   //Turn off LEFT LED

	if (DR == 1)
		GPIOE->BSRR |= 1 << 15;   //Turn on RIGHT LED
	else
		GPIOE->BRR |= 1 << 15;   //Turn off RIGHT LED

	TIM1->CCR2 = (uint32_t) abs(4095 * oct0 - abs(X * a) - abs(Y * b)); //Left PWM
	TIM1->CCR1 = (uint32_t) abs(4095 * oct1 - abs(X * p) - abs(Y * q)); //Right PWM
}

int mapp(float k, float l, float h, float L, float H) {
	return ((k - l) / (h - l)) * (H - L) + L;
}

void MotorCode(int x, int y) {
	x = mapp(x, 0, 4095, -4095, 4095);
	y = mapp(y, 0, 4095, -4095, 4095);

	if (abs(x) < 600)
		x = 0;

	else if (x < -3900)
		x = -4095;

	else if (x > 3900)
		x = 4095;

	if (abs(y) < 600)
		y = 0;

	else if (y < -3900)
		y = -4095;

	else if (y > 3900)
		y = 4095;

	if (abs(x) < 20 && abs(y) < 20)   //No Motion
		Drive(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	else if (abs(x) < 10 && y < 0)   //Full Forward
		Drive(1, 1, 0, 0, 1, 0, 0, 1, x, y);

	else if (abs(x) < 10 && y > 0)   //Full Backward
		Drive(0, 0, 0, 0, 1, 0, 0, 1, x, y);

	else if (x < 0 && abs(y) <= 20)   //Spot Turn Left
		Drive(0, 1, 0, 1, 0, 0, 1, 0, x, y);

	else if (x > 0 && abs(y) <= 20)   //Spot Turn Right
		Drive(1, 0, 0, 1, 0, 0, 1, 0, x, y);

	else if (x > 0 && y < 0 && x >= abs(y))   //Octet 1
			{
		if (abs(x) > 4095 - abs(y))
			Drive(1, 0, 0, 1, 0, 1, 0, 1, x, y);
		else
			Drive(1, 0, 1, 0, 1, 0, 1, 0, x, y);
	}

	else if (x > 0 && y < 0 && x < abs(y))   //Octet 2
			{
		if (abs(y) > 4095 - abs(x))
			Drive(1, 1, 0, 0, 1, 1, 1, 0, x, y);
		else
			Drive(1, 1, 1, 1, 0, 0, 0, 1, x, y);
	}

	else if (x < 0 && y < 0 && abs(y) > abs(x))   //Octet 3
			{
		if (abs(y) > 4095 - abs(x))
			Drive(1, 1, 1, 1, 0, 0, 0, 1, x, y);
		else
			Drive(1, 1, 0, 0, 1, 1, 1, 0, x, y);
	}

	else if (x < 0 && y < 0 && abs(x) >= abs(y))   //Octet 4
			{
		if (abs(x) > 4095 - abs(y))
			Drive(0, 1, 1, 0, 1, 0, 1, 0, x, y);
		else
			Drive(0, 1, 0, 1, 0, 1, 0, 1, x, y);
	}

	else if (x < 0 && y > 0 && abs(x) > abs(y))   //Octet 5
			{
		if (abs(x) > 4095 - abs(y))
			Drive(0, 1, 0, 1, 0, 1, 0, 1, x, y);
		else
			Drive(0, 1, 1, 0, 1, 0, 1, 0, x, y);
	}

	else if (x < 0 && y > 0 && abs(y) >= abs(x))   //Octet 6
			{
		if (abs(y) > 4095 - abs(x))
			Drive(0, 0, 0, 0, 1, 1, 1, 0, x, y);
		else
			Drive(0, 0, 1, 1, 0, 0, 0, 1, x, y);
	}

	else if (x > 0 && y > 0 && abs(y) >= abs(x))   //Octet 7
			{
		if (abs(y) > 4095 - abs(x))
			Drive(0, 0, 1, 1, 0, 0, 0, 1, x, y);
		else
			Drive(0, 0, 0, 0, 1, 1, 1, 0, x, y);
	}

	else if (x > 0 && y > 0 && abs(x) > abs(y))   //Octet 8
			{
		if (abs(x) > 4095 - abs(y))
			Drive(1, 0, 1, 0, 1, 0, 1, 0, x, y);
		else
			Drive(1, 0, 0, 1, 0, 1, 0, 1, x, y);
	}
}
void Error_Handler(void) {
	;
}

