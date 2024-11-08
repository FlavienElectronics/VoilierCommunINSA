#include "driver.h"
#include "application.h"

/*
Allumage de la LED verte embarqu�e
*/
void LED_ON(void){
		GPIO_Pin_Set(GPIOA, PIN_5);
}

/*
Extinction de la LED verte embarqu�e
*/
void LED_OFF(void){
		GPIO_Pin_Reset(GPIOA, PIN_5);
}

void LED_TogglePA5(void){
		if (GPIO_PIN_Read(PORT_A, PIN_5) == 1){
				LED_OFF();
		}else{
				LED_ON();
		}
}

void LED_TogglePC8(void){
		if (GPIO_PIN_Read(PORT_C, PIN_8) == 0){
				GPIO_Pin_Set(GPIOC, PIN_8);
		}else{
				GPIO_Pin_Reset(GPIOC, PIN_8);
		}
}
