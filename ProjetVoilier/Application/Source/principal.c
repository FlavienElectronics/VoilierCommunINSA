#include "stm32f10x.h"	 // Pour la gestion de la carte STM32
#include "application.h" // Contient les fonctions de haut niveau
#include "driver.h"		 // Contient les macros des ports et pins
#include <stdio.h>

extern char I2C_Error;

int main(void)
{

	// Initialisation de la pin GPIO de la LED (Output push-pull)
	GPIO_PIN_Init(PORT_A, PIN_5, OUTPUT_10MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);

	// Initialisation de la PIN A8 pour PWM du plateau
	GPIO_PIN_Init(PORT_A, PIN_8, OUTPUT_50MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);

	// Initialisation des PIN A2 et A3 pour la communication UART1
	GPIO_PIN_Init(PORT_A, PIN_2, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL); // PA2 en mode alternatif pour TX
	GPIO_PIN_Init(PORT_A, PIN_3, INPUT, FLOATING_INPUT);							 // PA3 en entrée flottante pour RX

	// Démarrage des TIMERS 1,2 et 3
	TIMER_init(TIM1);  // TIMER 1 : PWM du moteur
	TIMER_init(TIM2);  // TIMER 2 : Codeur incrémental de la girouette
	PWM_Init(TIM1, 0); // Initialisation de la PWM du moteur

	// Initialisation et mise en marche des UARTS 1 et 2
	USART1_init();
	USART2_init();

	// Initialisation de la PIN C12 pour le contrôle du sens du moteur
	GPIO_PIN_Init(PORT_C, PIN_12, OUTPUT_50MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);
	GPIO_PIN_Config(PORT_C, PIN_12, 0);

	// Initialisation des PIN A2 et A3 pour la communication UART2
	GPIO_PIN_Init(PORT_A, PIN_9, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
	GPIO_PIN_Init(PORT_A, PIN_10, INPUT, FLOATING_INPUT);

	// Initialisation des PIN pour les signaux de la girouette
	GPIO_PIN_Init(PORT_A, PIN_0, INPUT, FLOATING_INPUT);  // GIROUETTE CHA
	GPIO_PIN_Init(PORT_A, PIN_1, INPUT, FLOATING_INPUT);  // GIROUETTE CHB
	GPIO_PIN_Init(PORT_C, PIN_10, INPUT, FLOATING_INPUT); // GIROUETTE INDEX

	// Broche PWM de sortie pour contrôle servo moteur
	PWM3_Init(50); // PWM du servo-moteur

	Welcome();

	// Déclaration de l'interruption sur PC10 pour l'index de la girouette
	GPIO_EXTI_PC10_Init();

	// Démarrage ADC : PIN C0 pour la lecture de la tension de la batterie
	Start_ADC1(PORT_C, PIN_4);

	// Initialisation et démarrage du codeur incrémental
	ENCODEUR_init(TIM2);

	MySPI_Init(SPI1); // Initialisation SPI
	ADXL345_Init();	  // Initialisation ADXL345

	MyI2C_Init(I2C1, 2, I2C_Error_Callback); // Initialiser I2C1 avec la fonction callback d'erreur

	// Initialisation du DS1307
	DS1307_Init();

	int risqueChavirement = 0;

	while (1)
	{
		char reception = USART1_Receive();

		// 100->1    : GAUCHE
		// 255->156  : DROITE

		int tmp = (256 - (int)reception);

		int bat = BatteryRead();

		int val = TIM2->CNT;

		if (reception < 101 & reception != 0)
		{
			PWM_Set(TIM1, reception);
			GPIO_Pin_Reset(GPIOC, PIN_12);
		}
		else if (reception > 155)
		{
			PWM_Set(TIM1, tmp);
			GPIO_Pin_Set(GPIOC, PIN_12);
		}
		else
		{
			PWM_Set(TIM1, 0);
		}

		if (risqueChavirement == 0)
			PWM_Servo_Set((100 * val) / (1400));

		char buffer_batterie[32];
		sprintf(buffer_batterie, "Charge batterie : %d pourcent", bat);
		USART1_Transmit(buffer_batterie);
		USART1_Transmit("\n\r");

		if (I2C_Error)
		{
			USART2_Transmit("Erreur I2C détectée !\n");
			I2C_Error = 0; // Réinitialiser le flag d'erreur
		}
		else
		{
			Display_Time(); // Lire et afficher l'heure
		}

		risqueChavirement = ADXL345_anti_chavirement();
	}
}