#include "stm32f10x.h"	 // Pour la gestion de la carte STM32
#include "application.h" // Contient les fonctions de haut niveau
#include "driver.h"		 // Contient les macros des ports et pins
#include <stdio.h>

extern char I2C_Error;

/*
Projet Voilier - INSA Toulouse 2024/2025

Quadrinome :

						CARVALHO Flavien
						LESPIAUCQ Denis
						LE BEL Augustin
						PICARD Christophe

Groupe 4AE-SE3

*/

int main(void)
{
	// Initialisation de la pin GPIO de la LED (Output push-pull)
	GPIO_PIN_Init(PORT_A, PIN_5, OUTPUT_10MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);

	// Initialisation de la PIN A8 pour PWM du plateau
	GPIO_PIN_Init(PORT_A, PIN_8, OUTPUT_50MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);

	// Initialisation des PIN A2 et A3 pour la communication UART1
	GPIO_PIN_Init(PORT_A, PIN_2, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL); 	// PA2 en mode alternatif pour TX
	GPIO_PIN_Init(PORT_A, PIN_3, INPUT, FLOATING_INPUT);							 								// PA3 en entree flottante pour RX

	// Demarrage des TIMERS 1,2 et 3
	TIMER_init(TIM1);  // TIMER 1 : PWM du moteur
	TIMER_init(TIM2);  // TIMER 2 : Codeur incremental de la girouette
	PWM_Init(TIM1, 0); // Initialisation de la PWM du moteur

	// Initialisation et mise en marche des UARTS 1 et 2
	USART1_init();
	USART2_init();

	// Initialisation de la PIN C12 pour le contrele du sens du moteur
	GPIO_PIN_Init(PORT_C, PIN_12, OUTPUT_50MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);
	GPIO_PIN_Config(PORT_C, PIN_12, 0);

	// Initialisation des PIN A2 et A3 pour la communication UART2
	GPIO_PIN_Init(PORT_A, PIN_9, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
	GPIO_PIN_Init(PORT_A, PIN_10, INPUT, FLOATING_INPUT);

	// Initialisation des PIN pour les signaux de la girouette
	GPIO_PIN_Init(PORT_A, PIN_0, INPUT, FLOATING_INPUT);  // GIROUETTE CHA
	GPIO_PIN_Init(PORT_A, PIN_1, INPUT, FLOATING_INPUT);  // GIROUETTE CHB
	GPIO_PIN_Init(PORT_C, PIN_10, INPUT, FLOATING_INPUT); // GIROUETTE INDEX

	PWM3_Init(50); // PWM du servo-moteur

	Welcome(); // Clignottement de la LED verte

	GPIO_EXTI_PC10_Init(); // Declaration de l'interruption sur PC10 pour l'index de la girouette
	
	Start_ADC1(PORT_C, PIN_4); // Demarrage ADC : PIN C0 pour la lecture de la tension de la batterie

	CODEUR_init(TIM2); // Initialisation et demarrage du codeur incremental

	MySPI_Init(SPI1); // Initialisation SPI
	ADXL345_Init();	  // Initialisation ADXL345

	MyI2C_Init(I2C1, 2, I2C_Error_Callback); // Initialiser I2C1 avec la fonction callback d'erreur

	DS1307_Init(); // Initialisation du DS1307

	int risqueChavirement = 0; // Indique si l'angle du navire depasse la limite

	while (1)
	{
		/* 1- Gestion du dialogue Xbee*/
		char reception = USART1_Receive();
		int tmp = (256 - (int)reception);	// 100->1    : GAUCHE 	/ 	255->156  : DROITE
		/* 1- END*/

		/* 2- Lecture et affichage du niveau de batterie */
		int bat = BatteryRead();
		char buffer_batterie[32];
		sprintf(buffer_batterie, "Charge batterie : %d pourcent", bat);
		USART1_Transmit(buffer_batterie);
		USART1_Transmit("\n\r");
		/* 2- END */

		/* 3- Gestion du sens de rotation de la plateforme tournante */
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
		/* 3- END */

		/* 4- Gestion de la girouette */
		int val = TIM2->CNT;
		/* 4- END */
		
		/* 5- Gestion du servo moteur lie au bordage des voiles */
		if (risqueChavirement == 0)
			PWM_Servo_Set((100 * val) / (1400));
		/* 5- END */

		/* 6- Gestion de l'heure avec la RTC */
		if (I2C_Error)
		{
			USART2_Transmit("Erreur I2C detectee !\n");
			I2C_Error = 0; // Reinitialiser le flag d'erreur
		}
		else
		{
			Display_Time(); // Lire et afficher l'heure
		}
		/* 6- END */

		/* 7- Gestion du risque de chavirement du navire avec la IMU*/
		risqueChavirement = ADXL345_anti_chavirement();
		/* 7- END*/
	}
}