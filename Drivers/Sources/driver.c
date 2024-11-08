#include "../../STM/stm32f10x.h"
#include "../../STM/core_cm3.h"
#include "../Include/driver.h"
#include "stdint.h"

/* ========== REGION : CLOCK & TIMER ========== */

// Initialisation de toutes les CLOCKS de reference des GPIO (A, B et C) => STM32 103
void Clock_Init_All()
{
	RCC->APB2ENR |= RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN;
}

// Fonction permettant d'activer la CLOCK d'un GPIO
int Clock_Set(GPIO_TypeDef *GPIO)
{
	switch ((int)GPIO)
	{
	case (int)GPIOA:
		RCC->APB2ENR |= RCC_IOPAEN;
		break;
	case (int)GPIOB:
		RCC->APB2ENR |= RCC_IOPBEN;
		break;
	case (int)GPIOC:
		RCC->APB2ENR |= RCC_IOPCEN;
		break;
	case (int)GPIOD:
		RCC->APB2ENR |= RCC_IOPDEN;
		break;
	case (int)GPIOE:
		RCC->APB2ENR |= RCC_IOPEEN;
		break;
	default:
		return (-1);
	}
	return 0;
}

// Fonction permettant de désactiver la CLOCK d'un GPIO
int Clock_Reset(GPIO_TypeDef *GPIO)
{
	switch ((int)GPIO)
	{
	case (int)GPIOA:
		RCC->APB2ENR &= ~RCC_IOPAEN;
		break;
	case (int)GPIOB:
		RCC->APB2ENR &= ~RCC_IOPBEN;
		break;
	case (int)GPIOC:
		RCC->APB2ENR &= ~RCC_IOPCEN;
		break;
	case (int)GPIOD:
		RCC->APB2ENR &= ~RCC_IOPDEN;
		break;
	case (int)GPIOE:
		RCC->APB2ENR &= ~RCC_IOPEEN;
		break;
	default:
		return (-1);
	}
	return 0;
}

/*
0 -> NOK
else -> OK
Return 0 if the given address of timer is not valid
Return the index of the timer if the given address of timer is valid
*/
int TIMER_exist(TIM_TypeDef *timerAddress)
{
	/*
	Example : TIMER_Address[4] == TIM4_BASE
	ATTENTION : NO TIMER 0
	It goes from 1 to 17
	*/
	const uint32_t TIMER_Address[] = {0, TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE, TIM9_BASE,
									  TIM10_BASE, TIM11_BASE, TIM12_BASE, TIM13_BASE, TIM14_BASE, TIM15_BASE, TIM16_BASE, TIM17_BASE};

	uint32_t timerAddressLocal = (uint32_t)(timerAddress);
	int i = 0;
	while (i < 18 && timerAddressLocal != TIMER_Address[i])
	{
		i++;
	}
	if (i == 18) // If i == 18, the address of the timer doesn't exist
		return (0);
	return (i);
}

/*
		P�riode PSC = P�riode_Horloge * (PSC + 1)
		P�riode TIMER = P�riode_PSC * (ARR + 1) = P�riode_Horloge * (PSC + 1) * (ARR + 1)
*/

int TIMER_Set(TIM_TypeDef *timerAddress, uint16_t ARR_value, int _prescaler)
{
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	// Enable clocks before configuring
	switch (index)
	{
	case 1: // TIM1
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		break;
	case 2: // TIM2
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		break;
	case 3: // TIM3
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		break;
	case 4: // TIM4
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	default:
		return -1;
	}
	// Configure the registers
	if (_prescaler == -1){
		timerAddress->PSC = PRESCALER;
	}else{
		timerAddress->PSC = _prescaler;
	}
	timerAddress->ARR = ARR_value;
	timerAddress->CNT = 0;
	timerAddress->DIER |= (0x01 << 0);
	timerAddress->CR1 |= TIM_CR1_CEN;

	return 0;
}

int TIMER_Reset(TIM_TypeDef *timerAddress)
{
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	timerAddress->CR1 &= ~TIM_CR1_CEN;
	return 0;
}

uint16_t GET_ARR(int frequency)
{
	uint16_t ARR = (FREQUENCE_STM32 / ((PRESCALER + 1) * frequency)) - 1;
	return ARR;
}

int NVIC_Enable(TIM_TypeDef *timerAddress, int priority_value)
{
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	if ((priority_value >= 0) && (priority_value <= 15))
	{
		switch (index)
		{
		case 2: // TIM2
			NVIC_SetPriority(TIM2_IRQn, priority_value);
			NVIC_EnableIRQ(TIM2_IRQn);
			break;
		case 3: // TIM3
			NVIC_SetPriority(TIM3_IRQn, priority_value);
			NVIC_EnableIRQ(TIM3_IRQn);
			break;
		case 4: // TIM4
			NVIC_SetPriority(TIM4_IRQn, priority_value);
			NVIC_EnableIRQ(TIM4_IRQn);
			break;
		default:
			return (-2);
		}
	}
	else
	{
		return (-1);
	}
	NVIC->ISER[0] = (0x01 << (44 - 16));
	return 0;
}

int ResetInterruptFlag(TIM_TypeDef *timerAddress)
{
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	switch (index)
	{
	case 1:
		TIM1->SR &= ~(0x01 << 0);
		break;
	case 2: // TIM2
		TIM2->SR &= ~(0x01 << 0);
		break;
	case 3:
		TIM3->SR &= ~(0x01 << 0);
		break;
	case 4:
		TIM4->SR &= ~(0x01 << 0);
		break;
	default:
		return (-1);
	}
	return 0;
}

int NVIC_Disable(TIM_TypeDef *timerAddress)
{
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	switch (index)
	{
	case 2: // TIM2
		NVIC_DisableIRQ(TIM2_IRQn);
		break;
	case 3: // TIM3
		NVIC_DisableIRQ(TIM3_IRQn);
		break;
	case 4: // TIM4
		NVIC_DisableIRQ(TIM4_IRQn);
		break;
	default:
		return (-2);
	}
	return 0;
}

int PWM_Init(TIM_TypeDef *timerAddress, int duty){
	int index = TIMER_exist(timerAddress);
	if (duty >= 0 && duty <= 100 && index != 0){
			int my_arr = 719;
			//SET TIMER
			TIMER_Set(timerAddress, my_arr, 0);
			//AFFECTATION DUTY
			timerAddress->CCR1 = GET_ARR_Duty(my_arr, duty);
			timerAddress->CCMR1 &= ~(0xFF);
			timerAddress->CCMR1 |= (6 << 4);
			timerAddress->CCMR1 |= (1 << 3);
			timerAddress->CCER |= (1 << 0);
			timerAddress->BDTR |= (1 << 15);
			return 0;
	}else{
			return (-1);
	}
}

int PWM_Set(TIM_TypeDef *timerAddress, int duty){
	int index = TIMER_exist(timerAddress);
	if (duty >= 0 && duty <= 100 && index != 0){
		timerAddress->CCR1 = GET_ARR_Duty(719, duty);
		return 0;
	}else{
		return (-1);
	}
}

uint16_t GET_ARR_Duty(int _arr, int _duty){
	uint16_t duty_arr;
	if (_duty == 0){
		duty_arr = 0;
	}else{
		duty_arr = _arr*_duty/100;
	}
	return duty_arr;
}

/* ========== REGION : GPIO ========== */

// Assigne un pointeur au GPIO correspondant : Retourne -1 si le port n'existe pas, sion 0 si le processus a ete effectue avec succes.
int GPIO_Assignment(GPIO_TypeDef **GPIO, int port)
{
	switch (port)
	{
	case PORT_A:
		*GPIO = GPIOA;
		break;
	case PORT_B:
		*GPIO = GPIOB;
		break;
	case PORT_C:
		*GPIO = GPIOC;
		break;
	case PORT_D:
		*GPIO = GPIOD;
		break;
	case PORT_E:
		*GPIO = GPIOE;
		break;
	default:
		return (-1);
	}
	return (0);
}

// Configure le mode du GPIO : Retourne -1 si le port n'existe pas, sion 0 si le processus a ete effectue avec succes.
int GPIO_SetPinConfig(GPIO_TypeDef *GPIO, int pin, int mode, int configuration)
{
	int mask = ~0;
	int config_value = configuration << 2 | mode; // [CNF : MODE]

	if (GPIO == GPIOA)
	{
		Clock_Set(GPIOA);
	}
	else if (GPIO == GPIOB)
	{
		Clock_Set(GPIOB);
	}
	else if (GPIO == GPIOC)
	{
		Clock_Set(GPIOC);
	}

	if (pin < 0 || pin > 15)
		return (-1);

	if (configuration == INPUT_PULL_DOWN)
	{
		config_value = CONFIGURATION_PULL_UP_DOWN << 2 | mode;
		mask = ~(0x01 << pin); // PULL DOWN : BIT PxODR � 0
		GPIO->ODR &= mask;
	}
	else if (configuration == INPUT_PULL_UP)
	{
		config_value = CONFIGURATION_PULL_UP_DOWN << 2 | mode;
		mask = (0x01 << pin); // PULL UP : BIT PxODR � 1
		GPIO->ODR |= mask;
	}
	else if (configuration > INPUT_PULL_UP)
	{
		return (-1);
	}

	if (pin > 7)
	{
		mask = (int)(~0) & (int)(~(0xF << 4 * (pin - 8)));
		GPIO->CRH &= (uint32_t)mask;
		config_value <<= ((int)(pin - 8) * 4); // Shifting to the correct position
		GPIO->CRH |= (uint32_t)config_value;
	}
	else
	{
		mask = (int)(~0) & (int)(~(0xF << 4 * pin));
		GPIO->CRL &= (uint32_t)mask;
		config_value <<= (pin * 4); // Shifting to the correct position
		GPIO->CRL |= (uint32_t)config_value;
	}

	return (0);
}

// Mise a jour du mode (SET/RESET) du GPIO
int GPIO_Pin_Set(GPIO_TypeDef *GPIO, int pin)
{
	GPIO->BSRR = 0x01 << pin;
	return 0;
}

int GPIO_Pin_Reset(GPIO_TypeDef *GPIO, int pin)
{
	GPIO->BRR = 0x01 << pin;
	return 0;
}

// Lecture de l'etat du GPIO : Retourne le bit en question (1 ou 0)
int GPIO_ReadPin(GPIO_TypeDef *GPIO, int pin)
{
	return ((GPIO->IDR >> pin) & 0x01);
}

/*
Lecture de l'etat du GPIO
*/
int GPIO_PIN_Read(int port, int pin)
{
	GPIO_TypeDef *GPIO = 0;
	if (GPIO_Assignment(&GPIO, port) == -1)
		return (-1);

	return GPIO_ReadPin(GPIO, pin);
}

/* ========== REGION : CLOCK & TIMER ========== */

int Clock_Config(int config, GPIO_TypeDef *GPIO)
{
	if (config == SET)
	{
		Clock_Set(GPIO);
	}
	else if (config == RESET)
	{
		Clock_Reset(GPIO);
	}
	else
	{
		return (-1);
	}
	return 0;
}
/*
config == SET or RESET
*/
int TIMER_Config(TIM_TypeDef *timerAddress, int config, int frequency)
{
	uint16_t ARR_value = GET_ARR(frequency);
	switch (config)
	{
	case SET:
		TIMER_Set(timerAddress, ARR_value, -1);
		break;
	case RESET:
		TIMER_Reset(timerAddress);
		break;
	default:
		return (-1);
	}
	return 0;
}

/* ========== REGION : GPIO ========== */

/*
Initialise le pin du GPIO
*/
int GPIO_PIN_Init(int port, int pin, int mode, int config)
{
	GPIO_TypeDef *GPIO = 0;
	if (GPIO_Assignment(&GPIO, port) == -1)
		return (-1);

	return GPIO_SetPinConfig(GPIO, pin, mode, config);
}

/*
Mise a jour du GPIO en SET (1)
*/
int GPIO_PIN_Config(int port, int pin, int conf)
{
	GPIO_TypeDef *GPIO = 0;
	if (GPIO_Assignment(&GPIO, port) == -1)
		return (-1);

	switch (conf)
	{
	case SET:
		GPIO_Pin_Set(GPIO, pin);
		break;
	case RESET:
		GPIO_Pin_Reset(GPIO, pin);
		break;
	default:
		return (-1);
	}
	return 0;
}

/*
// Function pointer for interruption
struct Interrupt_Function_Pointer
{
	void (*TIM1_IT_Function)();
	void (*TIM2_IT_Function)();
	void (*TIM3_IT_Function)();
	void (*TIM4_IT_Function)();
};
struct Interrupt_Function_Pointer IT_Function;
*/

/*
 Function pointer for interruption
 Interrupt_Function_Pointer[0] -> Unused
 Interrupt_Function_Pointer[1] -> Interrupt fonction of TIM1
 Interrupt_Function_Pointer[4] -> Interrupt fonction of TIM4
 .
 .
 .
*/
void (*TIMER_Interrupt_Function_Pointer[])() = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0};

int TIMER_Interruption_Setup(TIM_TypeDef *timerAddress, int config, int frequency, void (*functionPointer)(void)){
	int index = TIMER_exist(timerAddress);
	if (index == 0)
		return -1;

	TIMER_Interrupt_Function_Pointer[index] = functionPointer;
	TIMER_Config(timerAddress, config, frequency);
	// Ajout du lien de l'interruption du TIMER 2
	NVIC_Enable(timerAddress, PRIORITY_HIGH);
	return (0);
}

void TIM1_IRQHandler(void)
{
	ResetInterruptFlag(TIM1);
	if (TIMER_Interrupt_Function_Pointer[1] != 0)
		(*TIMER_Interrupt_Function_Pointer[1])();
}

void TIM2_IRQHandler(void)
{
	ResetInterruptFlag(TIM2);
	if (TIMER_Interrupt_Function_Pointer[2] != 0)
		(*TIMER_Interrupt_Function_Pointer[2])();
}
void TIM3_IRQHandler(void)
{
	ResetInterruptFlag(TIM3);
	if (TIMER_Interrupt_Function_Pointer[3] != 0)
		(*TIMER_Interrupt_Function_Pointer[3])();
}

void TIM4_IRQHandler(void)
{
	ResetInterruptFlag(TIM4);
	if (TIMER_Interrupt_Function_Pointer[4] != 0)
		(*TIMER_Interrupt_Function_Pointer[4])();
}