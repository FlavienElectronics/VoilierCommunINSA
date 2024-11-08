#include "stm32f10x.h"   // Pour la gestion de la carte STM32
#include "application.h" // Contient les fonctions de haut niveau
#include "driver.h"      // Contient les macros des ports et pins

int main(void)
{
  // Clock_Init_All();

  // Initialisation de la pin GPIO de la LED (Output push-pull)
  GPIO_PIN_Init(PORT_A, PIN_5, OUTPUT_10MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);
  GPIO_PIN_Init(PORT_C, PIN_8, OUTPUT_10MHZ, GENERAL_PURPOSE_OUTPUT_PUSH_PULL);

  // Initialisation du bouton sur le GPIO PC13 (Floating input)
  GPIO_PIN_Init(PORT_C, PIN_13, INPUT, FLOATING_INPUT);
  GPIO_PIN_Init(PORT_C, PIN_10, INPUT, INPUT_PULL_UP);

  // D�marrage du TIMER 2 pour une fr�quence de 2 secondes = 0,5 s

  //TIMER_Interruption_Setup(TIM2, SET, 2, LED_TogglePA5);

//Uniquement timer 2 et 3 fonctionnels
  TIMER_Interruption_Setup(TIM2, SET, 2, LED_TogglePC8);

  TIMER_Interruption_Setup(TIM3, SET, 4, LED_TogglePA5);

  /*RESERVATION OF PA8 FOR PWM WITH TIMER 1*/
  GPIO_PIN_Init(PORT_A, PIN_8, OUTPUT_50MHZ,ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL);
		
	
	GPIO_PIN_Init(PORT_A, PIN_0, INPUT, ANALOG_MODE);
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	ADC1->CR2 |= (0x01 << 0); // aDON à 1 = TURN ON
	
	ADC1->CR2 |= (0x01 << 17) | (0x01 << 18) | (0x01 << 19); // EXTSEL
		
	RCC->CFGR = (RCC->CFGR |(0x1 << 15)) & ~(0x1 << 14);	//PRESCALER ADC mis à 6
	
	uint16_t data_read;
	PWM_Init(TIM1, 20);
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
	while (1){
		ADC1->CR2 |= ADC_CR2_SWSTART; //Demarrage de conversion SWSTART
		
		while (!(((ADC1->SR) >> 1) & 0x01)){
		}
		
		data_read = ADC1->DR;
		if (data_read != 0){
			int call_of_duty = 100*data_read/4095;
			PWM_Set(TIM1, call_of_duty);
		}
	}
	
	PWM_Init(TIM1, 20);
	while (1){
			for (int i = 0; i <= 100; i++){
				PWM_Set(TIM1, i);
				for (int y = 0; y < 100000; y++){
					int a = 0;
					a = 10;
					a = y * 2;
				}
			}
	}

  while (1)
  {
    // Verification de la pression sur le bouton (logique inverse)
    /*if (GPIO_PIN_Read(PORT_C, PIN_13) == 0)
    {
        // Allumage de la LED VERTE
        LED_ON();
    }
    else
    {
        // Extinction de la LED VERTE
        LED_OFF();
    }
    if (GPIO_PIN_Read(PORT_C, PIN_10) == 0)
    {
        // Allumage de la LED ROUGE ext.
        GPIO_Pin_Set(GPIOC, PIN_8);
    }
    else
    {
        // Extinction de la LED ROUGE ext.
        GPIO_Pin_Reset(GPIOC, PIN_8);
    }*/
  }
}