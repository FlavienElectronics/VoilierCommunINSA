#include "stm32f10x.h"   // Pour la gestion de la carte STM32
#include "application.h" // Contient les fonctions de haut niveau
#include "driver.h"      // Contient les macros des ports et pins
#include <stdio.h>


#include "MySPI.h"
#include <math.h>
#include "MyI2C.h"

//=================================================
// Partie Anti-Chavirement 
//fonction lecture et ecriture via SPI
#define DS1307_ADDRESS     0x68    // Adresse I2C 7 bits du DS1307
#define DS1307_TIME_ADDR   0x00    // Adresse de départ pour l'heure (Secondes, Minutes, Heures)

#define ANGLE_CRITIQUE 40     // Seuil de roulis critique en degrés
#define GRAVITE 9.81
#define PWM_DUTY_VOILE_FERME 50  // Duty Cycle pour voiles fermées (en %)
#define PWM_DUTY_VOILE_OUVERTE 90 // Duty Cycle pour voiles ouvertes (en %)

void MySPI_WriteRegister(char reg, char value) {
    MySPI_Clear_NSS();
    MySPI_Send(reg);
    MySPI_Send(value);
    MySPI_Set_NSS();
}

int MySPI_ReadRegister(char reg) {
    int value;
    MySPI_Clear_NSS();
    MySPI_Send(reg | 0x80); // Lecture du registre
    value = MySPI_Read();
    MySPI_Set_NSS();
    return value;
}


//==================================================
// Fonction d'initialisation de l'ADXL345
void ADXL_Init(){
	MySPI_WriteRegister(POWER_CTL, 0x08);
  MySPI_WriteRegister(DATA_FORMAT, 0x0B);
	// BW_RATE configuré par défaut à 100 Hz
}



// Calcul de l'angle de roulis en degrés
int calculate_angle(int16_t accel_y, int16_t accel_z) {
    return atan2((double)accel_y, (double)accel_z) * (180.0 / 3.14);
}
void afficher_angle(int angle){
	printf("L'angle de roulis est de : %d ", angle) ;
}

char I2C_Error = 0;

/* Structure pour l'I2C */
MyI2C_RecSendData_Typedef I2C_SendData;
MyI2C_RecSendData_Typedef I2C_ReceiveData;

/* ==================== CALLBACK ERREUR ==================== */
void I2C_Error_Callback(void) {
    I2C_Error = 1;  // Flag d'erreur
    USART2_Transmit("Erreur I2C détectée !\n");
}

/* ==================== CONVERSION BCD -> DECIMAL ==================== */
uint8_t BCD_To_Decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* ==================== CONVERSION DECIMAL -> BCD ==================== */
uint8_t Decimal_To_BCD(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/* ==================== INITIALISATION DU DS1307 ==================== */
void DS1307_Init(void) {
    char initial_time[3];

    // Exemple : Initialiser l'heure à 12:30:00
    initial_time[0] = Decimal_To_BCD(0);   // Secondes : 00
    initial_time[1] = Decimal_To_BCD(30);  // Minutes : 30
    initial_time[2] = Decimal_To_BCD(12);  // Heures : 12 (format 24h)

    I2C_SendData.SlaveAdress7bits = DS1307_ADDRESS;
    I2C_SendData.Ptr_Data = initial_time;
    I2C_SendData.Nb_Data = 3;  // 3 octets : Secondes, Minutes, Heures

    MyI2C_PutString(I2C1, DS1307_TIME_ADDR, &I2C_SendData);
    USART2_Transmit("Heure initialisée à 12:30:00\n");
}

/* ==================== LECTURE DE L'HEURE ==================== */
void DS1307_ReadTime(char *hours, char *minutes, char *seconds) {
    char time_data[3] = {0};  // Buffer pour stocker les données

    I2C_ReceiveData.SlaveAdress7bits = DS1307_ADDRESS;
    I2C_ReceiveData.Ptr_Data = time_data;
    I2C_ReceiveData.Nb_Data = 3;  // Lire 3 octets : Secondes, Minutes, Heures

    MyI2C_GetString(I2C1, DS1307_TIME_ADDR, &I2C_ReceiveData);

    *seconds = BCD_To_Decimal(time_data[0] & 0x7F);  // Secondes (masquer le bit CH)
    *minutes = BCD_To_Decimal(time_data[1]);
    *hours   = BCD_To_Decimal(time_data[2] & 0x3F);  // Heures (format 24h)
}

/* ==================== AFFICHAGE DE L'HEURE ==================== */
void Display_Time(void) {
    char hours, minutes, seconds;
    char buffer[20];

    DS1307_ReadTime(&hours, &minutes, &seconds);

    // Formater et envoyer l'heure via USART2
    sprintf(buffer, "Heure: %02d:%02d:%02d\n\r", hours, minutes, seconds);
    USART2_Transmit(buffer);
}


// Lecture des données brutes de l'ADXL345
void lire_accel_adxl345(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    *accel_x = (MySPI_ReadRegister(DATAX1) << 8) | MySPI_ReadRegister(DATAX0);
    *accel_y = (MySPI_ReadRegister(DATAY1) << 8) | MySPI_ReadRegister(DATAY0);
    *accel_z = (MySPI_ReadRegister(DATAZ1) << 8) | MySPI_ReadRegister(DATAZ0);
}



// Relâcher les voiles si l'angle critique est dépassé
void relacher_voiles() {
	PWM3_Set((100*PWM_DUTY_VOILE_FERME)/(1400));// Ouvrir les voiles
}



int SPI_anti_chavirement(void)
{
			int16_t accel_x, accel_y, accel_z;
		
					// Lire les données de l'ADXL345
			accel_x = (MySPI_ReadRegister(DATAX1) << 8) | MySPI_ReadRegister(DATAX0);
			accel_y = (MySPI_ReadRegister(DATAY1) << 8) | MySPI_ReadRegister(DATAY0);
			accel_z = (MySPI_ReadRegister(DATAZ1) << 8) | MySPI_ReadRegister(DATAZ0);
		
			
			int angle = calculate_angle(accel_y, accel_z); // Calculer l'angle de roulis
			// Envoyer l'angle de roulis à la console XBee
			
			char message[50];
			//sprintf(message, "accel: y %d\n\r", accel_y);
			//USART2_Transmit(message);
			
			//sprintf(message, "accel:  z %d \n\r", accel_z);
			//UART2_Transmit(message);
		
			sprintf(message, "Angle de roulis: %d degres\n\r\n", angle);
			USART2_Transmit(message);
			// Contrôler les voiles en fonction de l'angle
			if (fabs(angle) > ANGLE_CRITIQUE) {
					PWM_Servo_Set(PWM_DUTY_VOILE_FERME);// Voiles relâchées
				return (-1);
			} 
			return(0);
}



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
	PWM3_Init(50);	// PWM du servo-moteur
				
	Welcome();
	
	// Déclaration de l'interruption sur PC10 pour l'index de la girouette
	GPIO_EXTI_PC10_Init();
	
	// Démarrage ADC : PIN C0 pour la lecture de la tension de la batterie
	Start_ADC1(PORT_C, PIN_4);
		
	// Initialisation et démarrage du codeur incrémental
	ENCODEUR_init(TIM2);
		
    MySPI_Init(SPI1);                 // Initialisation SPI
    ADXL_Init();                      // Initialisation ADXL345

		MyI2C_Init(I2C1, 2, I2C_Error_Callback);  // Initialiser I2C1 avec la fonction callback d'erreur

    // Initialisation du DS1307
    DS1307_Init();
		
		int prob = 0;
		
	while(1){
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
		}else if(reception > 155)
		{
			PWM_Set(TIM1, tmp);
			GPIO_Pin_Set(GPIOC, PIN_12);
		}else
		{
			PWM_Set(TIM1, 0);
		}
		
		if(prob == 0)
			PWM_Servo_Set((100*val)/(1400));
		
		char buffer_batterie[32];
		sprintf(buffer_batterie, "Charge batterie : %d pourcent", bat);
		USART1_Transmit(buffer_batterie);
		USART1_Transmit("\n\r");
			
        if (I2C_Error) {
            USART2_Transmit("Erreur I2C détectée !\n");
            I2C_Error = 0;  // Réinitialiser le flag d'erreur
        } else {
            Display_Time();  // Lire et afficher l'heure
        }
				
				prob = SPI_anti_chavirement();
				
	}
}