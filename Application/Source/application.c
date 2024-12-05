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

int BatteryRead(void){
	int data_read;
	data_read = Read_ADC1();
	if (data_read != 0){
		return 359*data_read/4095;
	}else{ return 0; }
}

void Welcome(void){
	USART2_Transmit("CARTE RESET\n\r");
	for (int i = 0; i < 5; i++){
		LED_ON();
		for (int y = 0; y < 100000; y++);
		LED_OFF();
		for (int y = 0; y < 100000; y++);
	}
}