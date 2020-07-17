# ReadMe Laboratorio 3

Para correr el programa localizador del robot se debe ejecutar el launch correspondiente _localization.launch_ 

Al iniciar la ejecuciión se bloquea hasta recibir el comando _run_ en el terminal. Esto permite hacer el cambio de mapa y reposicionar el robot en el simulador antes de comenzar la localización.

Una vez iniciado el robot busca su posición y al encontrarla dice "located" usando la interfaz de sonido. El programa sigue funcionando despues de esto. Puede volver a perder la posición si la nube de particulas se dispersa demasiado, pero suele volver a encontrarla pronto. Se notifica esto tambien mediante la interfaz de sonido.