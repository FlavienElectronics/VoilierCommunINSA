#include "stm32f10x.h"

// Specifications internes du STM32
#define FREQUENCE_STM32 72000000

// GPIO Ports
#define PORT_A 0x0A
#define PORT_B 0x0B
#define PORT_C 0x0C
#define PORT_D 0x0D
#define PORT_E 0x0E

// GPIO Pin
#define PIN_0 0
#define PIN_1 1
#define PIN_2 2
#define PIN_3 3
#define PIN_4 4
#define PIN_5 5
#define PIN_6 6
#define PIN_7 7
#define PIN_8 8
#define PIN_9 9
#define PIN_10 10
#define PIN_11 11
#define PIN_12 12
#define PIN_13 13
#define PIN_14 14
#define PIN_15 15

// GPIO Pin Modes
#define INPUT 0x00
#define OUTPUT_10MHZ 0x01
#define OUTPUT_2MHZ 0x02
#define OUTPUT_50MHZ 0x03

// GPIO INPUT Configuration
#define ANALOG_MODE 0x00
#define FLOATING_INPUT 0x01
#define INPUT_PULL_DOWN 0x02
#define INPUT_PULL_UP 0x03
#define CONFIGURATION_PULL_UP_DOWN 0x02

// GPIO OUTPUT Configuration
#define GENERAL_PURPOSE_OUTPUT_PUSH_PULL 0x00
#define GENERAL_PURPOSE_OUTPUT_OPEN_DRAIN 0x01
#define ALTERNATE_FUNCTION_OUTPUT_PUSH_PULL 0x02
#define ALTERNATE_FUNCTION_OUTPUT_OPEN_DRAIN 0x03

// CLOCK Values
#define RCC_IOPAEN (0x01 << 2)
#define RCC_IOPBEN (0x01 << 3)
#define RCC_IOPCEN (0x01 << 4)
#define RCC_IOPDEN (0x01 << 5)
#define RCC_IOPEEN (0x01 << 6)

// Valeurs internes
 #define SET						0x01
 #define RESET					0x02
/*
#define TIMER_1 0x01
#define TIMER_2 0x02
#define TIMER_3 0x03
#define TIMER_4 0x04*/
#define PRESCALER 7199
#define PRIORITY_MELENCHON -1951 // :P
#define PRIORITY_SYSTEM 0
#define PRIORITY_MASTER 1
#define PRIORITY_HIGH 2
#define PRIORITY_LOW 3
#define PRIORITY_LEGACY 5

// Function declarations
void Clock_Init_All(void);
int Clock_Set(GPIO_TypeDef *GPIO);
int Clock_Reset(GPIO_TypeDef *GPIO);
int Clock_Config(int config, GPIO_TypeDef *GPIO);

int GPIO_Assignment(GPIO_TypeDef **GPIO, int port);
int GPIO_SetPinConfig(GPIO_TypeDef *GPIO, int pin, int mode, int configuration);
int GPIO_Pin_Set(GPIO_TypeDef *GPIO, int pin);
int GPIO_Pin_Reset(GPIO_TypeDef *GPIO, int pin);
int GPIO_ReadPin(GPIO_TypeDef *GPIO, int pin);
int GPIO_PIN_Init(int port, int pin, int mode, int config);
int GPIO_PIN_Config(int port, int pin, int conf);
int GPIO_PIN_Read(int port, int pin);

uint16_t GET_ARR(int frequency);

int TIMER_Set(TIM_TypeDef *timerAddress, uint16_t ARR_value, int _prescaler);
int TIMER_Reset(TIM_TypeDef *timerAddress);
int TIMER_Config(TIM_TypeDef *timerAddress,int config, int frequency);
int TIMER_exist(TIM_TypeDef *timerAddress);
int TIMER_Interruption_Setup(TIM_TypeDef *timerAddress, int config, int frequency, void (*functionPointer)(void));

int NVIC_Enable(TIM_TypeDef *timerAddress, int priority_value);
int NVIC_Disable(TIM_TypeDef *timerAddress);

int PWM_Init(TIM_TypeDef *timerAddress, int duty);
int PWM_Set(TIM_TypeDef *timerAddress, int duty);
uint16_t GET_ARR_Duty(int _arr, int _duty);

int ResetInterruptFlag(TIM_TypeDef *timerAddress);