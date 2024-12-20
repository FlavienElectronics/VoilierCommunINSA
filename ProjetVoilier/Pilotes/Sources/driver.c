#include "driver.h"
#include "stdint.h"
#include <stdio.h>
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

void TIMER_init(TIM_TypeDef *timerAddress){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	timerAddress->CNT = 0;
	timerAddress->DIER |= (0x01 << 0);
	timerAddress->CR1 |= TIM_CR1_CEN;
}

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

void ENCODEUR_init(TIM_TypeDef *TIMx){
	// 2. Configurer les entrées TI1 et TI2 pour le codeur
	// a. Mapper les signaux sur TI1 et TI2
	TIMx->CCMR1 |= (1 << 0) | (1 << 8); // TI1FP1 et TI2FP2
	// b. Configurer les polarités (non inversées ici) et activer les canaux
	TIMx->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Polarité non inversée
	TIMx->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); // Activer les entrées

	// c. Configurer les filtres d'entrée si nécessaire (optionnel)
	TIMx->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F); // Pas de filtre (0000)

	// 3. Configurer le mode de capture/compare en mode codeur
	TIMx->SMCR &= ~TIM_SMCR_SMS; // Clear SMS bits
	TIMx->SMCR |= (3 << 0); // SMS = 011: Mode codeur sur TI1 et TI2

	// 4. Configurer l'auto-reload (ARR) pour définir la plage de comptage
	TIMx->ARR = 0xFFFF; // Maximal (16 bits)

	// 5. Initialiser le compteur à 0
	TIMx->CNT = 0;

	// 6. Activer le compteur
	TIMx->CR1 |= TIM_CR1_CEN;
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

void PWM3_Init(int duty){
	int my_arr = 59999;
	// Activer les horloges
RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Horloge GPIOB
RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Horloge TIM3
RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  // Horloge AFIO

// Configurer PB0 comme sortie alternative push-pull
GPIOB->CRL &= ~(0xF << (0 * 4)); // Efface la configuration pour PB0
GPIOB->CRL |= (0xB << (0 * 4)); // Mode 50 MHz, Alternate Function Output Push-Pull
TIM3->CCMR2 |= (6 << 4);       // PWM mode 1 sur CH3
TIM3->CCER |= TIM_CCER_CC3E;   // Active CH3
TIM3->PSC = 23;            // Prescaler: 72 MHz / 72 = 1 MHz
TIM3->ARR = my_arr;         // Période: 1 MHz / 20000 = 50 Hz
TIM3->CCR3 =  GET_ARR_Duty(my_arr, duty);
TIM3->CR1 |= TIM_CR1_CEN;      // Active le Timer

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

void PWM3_Set(int duty){
	TIM3->CCR3 = GET_ARR_Duty(59999, duty);
}

int PWM_Servo_Set(int duty){
	int res = 599 + duty*5400/100;
	TIM3->CCR3 = res;
	return res;
}

uint16_t GET_ARR_Duty(int _arr, int _duty){
	return _arr*_duty/100;
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
/*
void TIM4_IRQHandler(void)
{
	ResetInterruptFlag(TIM4);
	if (TIMER_Interrupt_Function_Pointer[4] != 0)
		(*TIMER_Interrupt_Function_Pointer[4])();
}
*/
/* ========== REGION : ADC ========== */

void Start_ADC1(int PORT, int PIN){
	GPIO_PIN_Init(PORT, PIN, INPUT, ANALOG_MODE);
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC1->SQR3 = 14;	// Entrée n°15 liée à la broche PC5
	ADC1->CR2 |= (0x01 << 0); // aDON à 1 = TURN ON
	ADC1->CR2 |= (0x01 << 17) | (0x01 << 18) | (0x01 << 19); // EXTSEL
	RCC->CFGR = (RCC->CFGR |(0x1 << 15)) & ~(0x1 << 14);	//PRESCALER ADC mis à 6
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
}

uint16_t Read_ADC1(){
	ADC1->CR2 |= ADC_CR2_SWSTART; //Demarrage de conversion SWSTART
	while (!(((ADC1->SR) >> 1) & 0x01)){}
	return ADC1->DR;
}

/* ========== REGION : USART ========== */

void USART2_init(){
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Activer l'horloge pour USART2 sur APB1
	//RCC->APB1ENR |= RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN;
	
	USART2->CR1 &= ~(USART_CR1_UE); // Désactivation USART pour configurer les registres
	USART2->CR1 &= ~(USART_CR1_M);  // Longueur de mot 8 bits
	USART2->CR2 &= ~(USART_CR2_STOP); // 1 bit de stop
	USART2->CR3 &= ~(USART_CR3_DMAR); // Désactiver DMAR si le DMA n'est pas utilisé
	USART2->BRR = 36000000 / 9600; // Définir le baud rate
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Activation transmission et réception
	USART2->CR1 |= USART_CR1_UE; // Activation USART après configuration
}

void USART1_init(){
	//RCC->APB1ENR |= RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
	
	USART1->CR1 &= ~(USART_CR1_UE); // Désactivation USART pour configurer les registres
	USART1->CR1 &= ~(USART_CR1_M);  // Longueur de mot 8 bits
	USART1->CR2 &= ~(USART_CR2_STOP); // 1 bit de stop
	USART1->CR3 &= ~(USART_CR3_DMAR); // Désactiver DMAR si le DMA n'est pas utilisé
	USART1->BRR = 72000000 / 9600; // Définir le baud rate
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; // Activation transmission et réception
	USART1->CR1 |= USART_CR1_UE; // Activation USART après configuration
}

void USART2_Transmit(char *data) {
    while (*data) {
        while (!(USART2->SR & USART_SR_TXE)); // Wait for TX buffer to be empty
        USART2->DR = *data++; // Transmit data
    }
}

char USART2_Receive(void) {
    while (!(USART2->SR & USART_SR_RXNE)); // Attendre que le registre de réception soit plein
    return (char)(USART2->DR & 0xFF); // Lire le caractère reçu (8 bits)
}

void USART1_Transmit(char *data) {
    while (*data) {
        while (!(USART1->SR & USART_SR_TXE)); // Wait for TX buffer to be empty
        USART1->DR = *data++; // Transmit data
    }
}

char USART1_Receive(void) {
		int tmp = 0;
    while (!(USART1->SR & USART_SR_RXNE) && (tmp < 100000)){ tmp++; }; // Attendre que le registre de réception soit plein
    if (tmp >= 100000){
			return '\0';
		}else{
			return (char)(USART1->DR & 0xFF); // Lire le caractère reçu (8 bits)
		}
}

/* ========== REGION : GPIO_INTERRUPTION ========== */

// Configuration de l'EXTI pour PC10
void GPIO_EXTI_PC10_Init(void) {
    // Activer l'horloge pour GPIOC et AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    // Mapper PC10 à EXTI10
    AFIO->EXTICR[2] &= ~(AFIO_EXTICR3_EXTI10); // Clear
    AFIO->EXTICR[2] |= (2 << 8); // GPIOC -> EXTI10

    // Configurer EXTI10 pour déclencher sur flanc descendant
    EXTI->IMR |= (1 << 10);  // Activer interruption pour EXTI10
    EXTI->EMR &= ~(1 << 10); // Désactiver événement (uniquement interruption)
    EXTI->RTSR &= ~(1 << 10); // Désactiver flanc montant
    EXTI->FTSR |= (1 << 10);  // Activer flanc descendant

    // Activer EXTI10 dans le NVIC
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 2); // Priorité d'interruption
}

// Gestionnaire d'interruption pour EXTI15_10 (PC10 est sur cette ligne)
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << 10)) { // Vérifier si EXTI10 est la source
        EXTI->PR |= (1 << 10); // Effacer le drapeau d'interruption
        			
				TIM2->CNT = 700;
    }
}

void MySPI_WriteRegister(char reg, char value)
{
	MySPI_Clear_NSS();
	MySPI_Send(reg);
	MySPI_Send(value);
	MySPI_Set_NSS();
}

int MySPI_ReadRegister(char reg)
{
	int value;
	MySPI_Clear_NSS();
	MySPI_Send(reg | 0x80); // Lecture du registre
	value = MySPI_Read();
	MySPI_Set_NSS();
	return value;
}


