#include "stm32f10x.h"   // Pour la gestion de la carte STM32
#include "application.h" // Contient les fonctions de haut niveau
#include "driver.h"      // Contient les macros des ports et pins
#include <stdio.h>

int main(void)
{

  // Initialisation de la pin GPIO de la LED (Output push-pull)
  GPIO_PIN_Init(PORT_A, PIN_5, OUTPUT_10MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);

  // Initialisation de la PIN A8 pour PWM du plateau
  GPIO_PIN_Init(PORT_A, PIN_8, OUTPUT_50MHZ,ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
	
	// Initialisation des PIN A2 et A3 pour la communication UART1
	GPIO_PIN_Init(PORT_A, PIN_2, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL); // PA2 en mode alternatif pour TX
	GPIO_PIN_Init(PORT_A, PIN_3, INPUT, FLOATING_INPUT); // PA3 en entrée flottante pour RX
	
	// Démarrage des TIMERS 1,2 et 3
	TIMER_init(TIM1);		// TIMER 1 : PWM du moteur
	TIMER_init(TIM2);		// TIMER 2 : Codeur incrémental de la girouette
	PWM_Init(TIM1, 0);	// Initialisation de la PWM du moteur
	
	
	// Initialisation et mise en marche des UARTS 1 et 2
	USART1_init();
	USART2_init();
	
	// Initialisation de la PIN C12 pour le contrôle du sens du moteur
	GPIO_PIN_Init(PORT_C, PIN_12, OUTPUT_50MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL );
	GPIO_PIN_Config(PORT_C, PIN_12, 0);
	
	// Initialisation des PIN A2 et A3 pour la communication UART2
	GPIO_PIN_Init(PORT_A, PIN_9, OUTPUT_10MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
	GPIO_PIN_Init(PORT_A, PIN_10, INPUT, FLOATING_INPUT);
	
	// Initialisation des PIN pour les signaux de la girouette
	GPIO_PIN_Init(PORT_A, PIN_0, INPUT, FLOATING_INPUT);		// GIROUETTE CHA
	GPIO_PIN_Init(PORT_A, PIN_1, INPUT, FLOATING_INPUT);		// GIROUETTE CHB
	GPIO_PIN_Init(PORT_C, PIN_10, INPUT, FLOATING_INPUT);		// GIROUETTE INDEX
	
	// Broche PWM de sortie pour contrôle servo moteur
	//GPIO_PIN_Init(PORT_B, PIN_0, OUTPUT_50MHZ, ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
	//TIMER_init(TIM3);		// TIMER 3 : PWM du servo
	PWM3_Init(50);	// PWM du servo-moteur
	//PWM3_Set(TIM3, 50);
				
	Welcome();
	
	// Déclaration de l'interruption sur PC10 pour l'index de la girouette
	GPIO_EXTI_PC10_Init();
	
	// Démarrage ADC : PIN C0 pour la lecture de la tension de la batterie
	Start_ADC1(PORT_C, PIN_4);
		
	// Initialisation et démarrage du codeur incrémental
	ENCODEUR_init(TIM2);
	

	
	while(1){
		char reception = USART1_Receive();
		/*char reception_cpy = reception;
		USART2_Transmit(" >> Valeur USART 1 : ");
		char buffer0[3];
		sprintf(buffer0,"%d",reception_cpy);
		USART2_Transmit(buffer0);
		USART2_Transmit("\n\r");*/
		
		// 100->1    : GAUCHE
		// 255->156  : DROITE
		
		int tmp = (256 - (int)reception);
			
		/*char buffer[32];
		sprintf(buffer, "tmp : %d\trec : %d", tmp, reception);
		USART2_Transmit(buffer);
		USART2_Transmit("\n\r");*/
		
		int bat = BatteryRead();
		
		int val = TIM2->CNT;
		/*USART2_Transmit(" >> Valeur compteur : ");
		char buffer_girouette[32];
		sprintf(buffer_girouette, "%d\t[%d]\t[%d pourcent]", val, GPIO_PIN_Read(PORT_C, PIN_10), bat);
		USART2_Transmit(buffer_girouette);
		USART2_Transmit("\n\r");*/

		if (reception < 101 & reception != 0)
		{
			PWM_Set(TIM1, reception);
			GPIO_Pin_Reset(GPIOC, PIN_12);
		}else if(reception > 155)
		{
			//USART2_Transmit("DROITE\n\r");
			PWM_Set(TIM1, tmp);
			GPIO_Pin_Set(GPIOC, PIN_12);
		}else
		{
			PWM_Set(TIM1, 0);
		}
		
		PWM_Servo_Set((100*val)/(1400));
		
		char buffer_batterie[32];
		sprintf(buffer_batterie, "Charge batterie : %d pourcent", bat);
		USART1_Transmit(buffer_batterie);
		USART1_Transmit("\n\r");
			
	
		/*char buffer1[32];
		sprintf(buffer1, "SERVO : %d\tVAL : %d", tmp0, val);
		USART2_Transmit(buffer1);
		USART2_Transmit("\n\r");*/
		
				
		/*if (val < 65536 && val > 65535/2){		//599 (1 %) et 5999 (10 %) - ENTRE 65535 et 64150
			//PWM3_Set(1);
			int tmp = PWM_Servo_Set(100*(val-64150)/1385);
			char buffer[32];
			sprintf(buffer, "SERVO (1) : %d", tmp);
			USART2_Transmit(buffer);
			USART2_Transmit("\n\r");
			LED_OFF();
		}else{																// ENTRE 0 et 1385
			//PWM3_Set(10);
			int tmp = PWM_Servo_Set(100*(val)/1385);
			char buffer[32];
			sprintf(buffer, "SERVO : %d", tmp);
			USART2_Transmit(buffer);
			USART2_Transmit("\n\r");
			LED_ON();
		}*/
	}
}