#ifndef MYSPI_C
#define MYSPI_C

#include "MySPI.h"
/*@brief Configure le SPI sp�cifi� : FSCK = 281kHz, Repos SCK = '1', Front actif = up
					 Gestion /CS logicielle � part, configure les 4 IO
					 - SCK, MOSI : Out Alt push pull
					 - MISO : floating input
					 - /NSS (/CS) : Out push pull
	* @param SPI_TypeDef * SPI : SPI1 ou SPI2 
  */
void MySPI_Init(SPI_TypeDef * SPI){
	
}



/*
  * @brief Envoie un octet (/CS non g�r�, � faire logiciellement)
					 Plus en d�tail, �mission de l'octet souhait� sur MOSI
					 Lecture en m�me temps d'un octet poubelle sur MISO (non exploit�)
  * @param : char ByteToSend : l'octet � envoyer
  */
void MySPI_Send(char ByteToSend){
	
}



/*
  * @brief Re�oit un octet (/CS non g�r�, � faire logiciellement)
					 Plus en d�tail, �mission d'un octet bidon sur MOSI (0x00)
					 pour �laborer les 8 fronts sur SCK et donc piloter le slave en lecture 
					 qui r�pond sur MISO
	* @param : none
	* @retval : l'octet lu.
  */
char MySPI_Read(void){
	
}


/*
  * @brief Positionne /CS = /NSS � '1'. A utiliser pour borner les octets � transmettre/recevoir
	* @param : none
  */
void MySPI_Set_NSS(void){
	
}



/*
  * @brief Positionne /CS = /NSS � '0'.  A utiliser pour borner les octets � transmettre/recevoir
	* @param :none
  */
void MySPI_Clear_NSS(void){
	
} 




#endif
