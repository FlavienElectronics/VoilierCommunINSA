#include "stm32f10x.h"
#include <math.h>

#include "driver.h"
#include "MySPI.h"
#include "MyI2C.h"

//=================================================
// Partie Anti-Chavirement
// fonction lecture et ecriture via SPI
#define DS1307_ADDRESS 0x68   // Adresse I2C 7 bits du DS1307
#define DS1307_TIME_ADDR 0x00 // Adresse de départ pour l'heure (Secondes, Minutes, Heures)

#define ANGLE_CRITIQUE 40 // Seuil de roulis critique en degrés
#define GRAVITE 9.81
#define PWM_DUTY_VOILE_FERME 50   // Duty Cycle pour voiles fermées (en %)
#define PWM_DUTY_VOILE_OUVERTE 90 // Duty Cycle pour voiles ouvertes (en %)

// Déclaration des fonctions utiles à l'utilisateur

/*Allumage de la LED verte embarquee*/
void LED_ON(void);

/*Extinction de la LED verte embarquee*/
void LED_OFF(void);

/* Fonction de lecture du niveau de batterie */
int BatteryRead(void);

/* Permet de faire clignoter la LED verte sur la carte pour distinguer un RESET */
void Welcome(void);

/* =============================== PARTIE ADXL345 (SPI) ============================= */

/* Fonction d'initialisation de l'ADXL345 */
void ADXL345_Init();

/* Lecture des données brutes de l'ADXL345 */
void ADXL345_lire_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

/*Fonction d'empechement de chavirement du voilier */
int ADXL345_anti_chavirement(void);

/*=============================== PARTIE DS1307 (I²C) =============================*/

/* Initialisation du DS1307 */
void DS1307_Init(void);

/* Lecture de l'heure */
void DS1307_ReadTime(char *hours, char *minutes, char *seconds);




/* Affichage de l'heure */
void Display_Time(void);

/* Calcul de l'angle de roulis en degres */
int calculate_angle(int16_t accel_y, int16_t accel_z);

/* Converti une valeur BCD en valeur decimale */
uint8_t BCD_To_Decimal(uint8_t bcd);

/* Converti une valeur decimale en valeur BCD */
uint8_t Decimal_To_BCD(uint8_t dec);

/* Relache les voiles si l'angle critique est depasse */
void relacher_voiles();

/* Affiche que la qu'une erreur s'est produire sur la liaison I2C*/
void I2C_Error_Callback(void);

