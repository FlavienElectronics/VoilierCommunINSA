#ifndef MYSPI_C
#define MYSPI_C

#include "MySPI.h"
/*@brief Configure le SPI spécifié : FSCK = 281kHz, Repos SCK = '1', Front actif = up
					 Gestion /CS logicielle à part, configure les 4 IO
					 - SCK, MOSI : Out Alt push pull
					 - MISO : floating input
					 - /NSS (/CS) : Out push pull
	* @param SPI_TypeDef * SPI : SPI1 ou SPI2 
  */
void MySPI_Init(SPI_TypeDef * SPI){
	
}



/*
  * @brief Envoie un octet (/CS non géré, à faire logiciellement)
					 Plus en détail, émission de l'octet souhaité sur MOSI
					 Lecture en même temps d'un octet poubelle sur MISO (non exploité)
  * @param : char ByteToSend : l'octet à envoyer
  */
void MySPI_Send(char ByteToSend){
	
}



/*
  * @brief Reçoit un octet (/CS non géré, à faire logiciellement)
					 Plus en détail, émission d'un octet bidon sur MOSI (0x00)
					 pour élaborer les 8 fronts sur SCK et donc piloter le slave en lecture 
					 qui répond sur MISO
	* @param : none
	* @retval : l'octet lu.
  */
char MySPI_Read(void){
	
}


/*
  * @brief Positionne /CS = /NSS à '1'. A utiliser pour borner les octets à transmettre/recevoir
	* @param : none
  */
void MySPI_Set_NSS(void){
	
}



/*
  * @brief Positionne /CS = /NSS à '0'.  A utiliser pour borner les octets à transmettre/recevoir
	* @param :none
  */
void MySPI_Clear_NSS(void){
	
} 




#endif
