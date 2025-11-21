AIT ALLA Hajar  

Bentekfa Maram

# TP de Synthèse – Autoradio

## 1 – Démarrage


### 1.1 Création du projet pour la carte NUCLEO_L476RG

Un projet a été créé sous **STM32CubeIDE** en sélectionnant la carte **NUCLEO_L476RG**.  
Tous les périphériques ont été initialisés avec leur **configuration par défaut**, conformément aux consignes, et la **BSP n’a pas été activée**.



### 1.2 Test de la LED LD2

La LED LD2 connectée à la broche **PA5** a été configurée comme une sortie GPIO.  
Un clignotement a été programmé afin de vérifier le bon fonctionnement de la carte.

```c
HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
HAL_Delay(100);
 while(1){
  printf("\r\nTP_Audio\r\n");
}
int __io_putchar(int chr)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
    return chr;
}
