#include "application.h"

char I2C_Error = 0;

/* Structure pour l'I2C */
MyI2C_RecSendData_Typedef I2C_SendData;
MyI2C_RecSendData_Typedef I2C_ReceiveData;

/*Allumage de la LED verte embarquee*/
void LED_ON(void)
{
	GPIO_Pin_Set(GPIOA, PIN_5);
}

/*Extinction de la LED verte embarquee*/
void LED_OFF(void)
{
	GPIO_Pin_Reset(GPIOA, PIN_5);
}

/* Fonction de lecture du niveau de batterie */
int BatteryRead(void)
{
	int data_read;
	data_read = Read_ADC1();
	if (data_read != 0)
	{
		return 359 * data_read / 4095;
	}
	else
	{
		return 0;
	}
}

/* Permet de faire clignoter la LED verte sur la carte pour distinguer un RESET */
void Welcome(void)
{
	USART2_Transmit("CARTE RESET\n\r");
	for (int i = 0; i < 5; i++)
	{
		LED_ON();
		for (int y = 0; y < 100000; y++)
			;
		LED_OFF();
		for (int y = 0; y < 100000; y++)
			;
	}
}

/* =============================== PARTIE ADXL345 (SPI) ============================= */
/* Fonction d'initialisation de l'ADXL345 */
void ADXL345_Init()
{
	MySPI_WriteRegister(POWER_CTL, 0x08);
	MySPI_WriteRegister(DATA_FORMAT, 0x0B);
	// BW_RATE configuré par défaut à 100 Hz
}
/* Lecture des données brutes de l'ADXL345 */
void ADXL345_lire_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
	*accel_x = (MySPI_ReadRegister(DATAX1) << 8) | MySPI_ReadRegister(DATAX0);
	*accel_y = (MySPI_ReadRegister(DATAY1) << 8) | MySPI_ReadRegister(DATAY0);
	*accel_z = (MySPI_ReadRegister(DATAZ1) << 8) | MySPI_ReadRegister(DATAZ0);
}

/*Fonction d'empechement de chavirement du voilier */
int ADXL345_anti_chavirement(void)
{
	int16_t accel_x, accel_y, accel_z;

	// Lire les données de l'ADXL345
	ADXL345_lire_accel(&accel_x, &accel_y, &accel_z);

	int angle = calculate_angle(accel_y, accel_z); // Calculer l'angle de roulis

	// Envoyer l'angle de roulis à la console XBee
	char message[50];
	sprintf(message, "accel: y %d\n\r", accel_y);
	USART1_Transmit(message);

	sprintf(message, "accel:  z %d \n\r", accel_z);
	USART1_Transmit(message);

	sprintf(message, "Angle de roulis: %d degres\n\r\n", angle);
	USART2_Transmit(message);

	// Contrôler les voiles en fonction de l'angle
	if (fabs(angle) > ANGLE_CRITIQUE)
	{
		PWM_Servo_Set(PWM_DUTY_VOILE_FERME); // Voiles relâchées
		return (-1);
	}
	return (0);
}

/*=============================== PARTIE DS1307 (I²C) =============================*/
/* Initialisation du DS1307 */
void DS1307_Init(void)
{
	char initial_time[3];

	// Exemple : Initialiser l'heure à 12:30:00
	initial_time[0] = Decimal_To_BCD(0);  // Secondes : 00
	initial_time[1] = Decimal_To_BCD(30); // Minutes : 30
	initial_time[2] = Decimal_To_BCD(12); // Heures : 12 (format 24h)

	I2C_SendData.SlaveAdress7bits = DS1307_ADDRESS;
	I2C_SendData.Ptr_Data = initial_time;
	I2C_SendData.Nb_Data = 3; // 3 octets : Secondes, Minutes, Heures

	MyI2C_PutString(I2C1, DS1307_TIME_ADDR, &I2C_SendData);
	USART2_Transmit("Heure initialisée à 12:30:00\n");
}

/* Lecture de l'heure */
void DS1307_ReadTime(char *hours, char *minutes, char *seconds)
{
	char time_data[3] = {0}; // Buffer pour stocker les données

	I2C_ReceiveData.SlaveAdress7bits = DS1307_ADDRESS;
	I2C_ReceiveData.Ptr_Data = time_data;
	I2C_ReceiveData.Nb_Data = 3; // Lire 3 octets : Secondes, Minutes, Heures

	MyI2C_GetString(I2C1, DS1307_TIME_ADDR, &I2C_ReceiveData);

	*seconds = BCD_To_Decimal(time_data[0] & 0x7F); // Secondes (masquer le bit CH)
	*minutes = BCD_To_Decimal(time_data[1]);
	*hours = BCD_To_Decimal(time_data[2] & 0x3F); // Heures (format 24h)
}


/* Affichage de l'heure */
void Display_Time(void)
{
	char hours, minutes, seconds;
	char buffer[20];

	DS1307_ReadTime(&hours, &minutes, &seconds);

	// Formater et envoyer l'heure via USART2
	sprintf(buffer, "Heure: %02d:%02d:%02d\n\r", hours, minutes, seconds);
	USART2_Transmit(buffer);
	USART1_Transmit(buffer);
}

/* Calcul de l'angle de roulis en degres */
int calculate_angle(int16_t accel_y, int16_t accel_z)
{
	return atan2((double)accel_y, (double)accel_z) * (180.0 / 3.14);
}

/* Converti une valeur BCD en valeur decimale */
uint8_t BCD_To_Decimal(uint8_t bcd)
{
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* Converti une valeur decimale en valeur BCD */
uint8_t Decimal_To_BCD(uint8_t dec)
{
	return ((dec / 10) << 4) | (dec % 10);
}

/* Relache les voiles si l'angle critique est depasse */
void relacher_voiles()
{
	PWM3_Set((100 * PWM_DUTY_VOILE_FERME) / (1400)); // Ouvrir les voiles
}

/* Affiche que la qu'une erreur s'est produire sur la liaison I2C*/
void I2C_Error_Callback(void)
{
	I2C_Error = 1; // Flag d'erreur
	USART2_Transmit("Erreur I2C détectée !\n");
}
