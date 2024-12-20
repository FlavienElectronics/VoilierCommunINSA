# ⛵ STM32 : Projet VOILIER

Bienvenue dans le projet **VOILIER** de l'INSA de Toulouse (2024/2025)

GROUPE 4AE-SE3

## 🚸 Equipe 1

    CARVALHO Flavien
    LESPIAUCQ Denis
            Cette équipe s'est occupée des parties suivantes :
                - Codeur incrémental (mesure du sens du vent avec la girouette)
                - Communication USART (reception module Xbee) 
                - Génération PWM et bit de sens (rotation de la plateforme)
                - Pont diviseur de tension (mesure batterie)
                - Servo moteur (bordage des voiles)

    
## 🚸 Equipe 2

    LE BEL Augustin
    PICARD Christophe
            Cette équipe s'est occupée des parties suivantes :
                - Inertial Measurement Unit (IMU, gestion de l'anti-chaviremment)
                - Real Time Clock (RTC, gestion de l'horodatage)
    

## 📁 Structure du projet

    📁 DRIVER
        📄driver.c
        📄driver.h
    📁 APPLICATION
        📄application.c
        📄application.h
    📁 VOILIER
        📄projet.uvprojx
    
    Le projet KEIL uVision 5 intègre automatiquement les bibliothèques DRIVER et APPLICATION utiles à l'utilisateur pour le contrôle du voilier.

Nous tenons à remercier M. Mahout qui a su nous conseiller et nous apporter sa bonne humeur tout au long ce projet.


