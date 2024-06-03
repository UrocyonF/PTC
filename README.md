# PTC

## Informations générales

L'objectif de ce projet est de réaliser un robot bibliothécaire, capable de se déplacer dans une bibliothèque et de localiser les livres sur les étagères.

Le robot doit pouvoir guider un utilisateur jusqu'à un livre donné, en lui indiquant le chemin à suivre. Puis faire un bruit et une lumière pour indiquer à l'utilisateur que le livre est à sa portée.

L'envirronement de travail est une bibliothèque, avec des étagères, des livres et possiblement des obstacles que le robot devra contourner.

## Broches utilisées

- UART1 : communication avec le Serializer
  - PC4 : TX
  - PC5 : RX
- UART2 : communication avec le Raspberry Pi
  - PA2 : TX
  - PA3 : RX
- PWM : contrôle des moteurs de la tourelle
  - PC6 : moteur horizontal
  - PB0 : moteur vertical
- GPIO : capteur à ultrasons
  - PB1 : trigger
  - PB2 : echo
- GPIO : contrôle des LEDs et du buzzer
  - PD3 : LED rouge
  - PD8 : buzzer
- GPIO : capteur photodiode
  - PC1 : capteur droite
  - PC2 : capteur gauche

## Comment compiler

Ce projet à été réalisé avec l'IDE STM32CubeIDE. Pour compiler le projet, il suffit de l'ouvrir avec l'IDE et de cliquer sur le bouton "Build" (flèche verte).

## Comment flasher

Pour flasher le microcontrôleur, il suffit de brancher le ST-Link sur le port SWD du microcontrôleur et de cliquer sur le bouton "Debug" (bug vert).

## Comment utiliser

Dans le [programme actuel](./Core/Src/main.c#359), le robot est contrôlé par un Raspberry Pi, qui envoie des commandes au microcontrôleur via UART2.

Si vous souhaitez tester d'autres programmes, il suffit de se rendre dans la [boucle principale du programme dans la fonction main](./Core/Src/main.c#173) et de décommenter une autre partie du code.
