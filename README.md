AIT ALLA Hajar  
Bentekfa Maram  

# TP de Synthèse – Autoradio

## 1 – Démarrage

### 1.1 Création du projet pour la carte NUCLEO_L476RG

Un projet a été créé sous **STM32CubeIDE** en sélectionnant la carte **NUCLEO_L476RG**.  
Tous les périphériques ont été initialisés avec leur configuration par défaut, et la BSP n’a pas été activée, conformément aux consignes du TP.

---

### 1.2 Test de la communication USART2 (via ST-Link)

L’interface **USART2** (connectée à la ST-Link interne) a été utilisée pour vérifier la communication entre la carte et le PC via **Tera Term**.

Un message de test a été envoyé depuis la carte :

```c
printf("Test printf sur USART2 !\r\n");
```
![image1](assets/image1)



### 1.3–1.4 Redirection de printf et validation de la communication USART2

Pour permettre l’utilisation de `printf` au sein d’un système multitâche et faciliter l’affichage des messages sur le terminal série, la redirection de la fonction `printf` et l’activation de FreeRTOS ont été réalisées conjointement.

#### Redirection de `printf` vers l’USART2

La fonction suivante redirige tous les appels à `printf()` vers l’USART2, utilisé comme port série via la ST-Link :

```c
int __io_putchar(int chr)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
    return chr;
}

```
![Tâche FreeRTOS Hello](assets/image5.jpg)

![FreeRTOS + printf](assets/image3.jpg)

- Les tâches FreeRTOS sont exécutées correctement  
- L’ordonnanceur fonctionne comme prévu
  
  ### 1.5 Configuration de FreeRTOS

Le système **FreeRTOS** a été activé en mode **CMSIS_V1** dans STM32CubeIDE.  
Cette configuration permet de gérer les tâches du projet via un ordonnanceur temps réel.

Les captures ci-dessous montrent :

- l’activation de FreeRTOS avec l’option **USE_NEWLIB_REENTRANT** (Newlib réentrante) ;
- l’utilisation de **TIM7** comme source de timebase pour le tick système.

![Configuration FreeRTOS CMSIS_V1](assets/image11.jpg)

![Sélection de TIM7 comme timebase](assets/image10.jpg)

### 1.6 Mise en place du Shell

Un **shell série** a été mis en place afin de permettre une interaction directe avec la carte via le terminal série.

Au démarrage, le message suivant est affiché :

===== Monsieur Shell v0.2 =====


![image4](assets/image4.jpg)

Ce message indique que le shell a été correctement initialisé et qu’il est prêt à recevoir des commandes.



Une première commande de test a été envoyée dans le terminal :


Le shell a alors répondu :

![image2](assets/image2.jpg)

Ce résultat confirme que :

- Le shell fonctionne correctement  
- Les commandes sont reconnues et traitées  
- Le système est prêt pour l’ajout de nouvelles commandes  

---
## 2 – Le GPIO Expander et le VU-Mètre

### 2.1 Configuration du GPIO Expander

Le VU-mètre du TP est piloté à l’aide d’un **GPIO Expander** dont la référence est :

- **MCP23S17** (expander GPIO 16 bits commandé en SPI)

La communication entre le STM32 et le MCP23S17 se fait via le bus **SPI3** du microcontrôleur.

### 2.2 Configuration du SPI3 dans STM32CubeIDE

#### 2.2.1 Paramètres généraux du SPI

D’après la documentation du **MCP23S17**, la fréquence maximale du bus SPI est de **10 MHz**.  
Nous avons donc configuré le périphérique **SPI3** à 10 MHz dans STM32CubeIDE, comme illustré ci-dessous :

![Configuration du SPI3 dans STM32CubeIDE](assets/image9.jpg)

Les paramètres principaux sont les suivants :

- **Fréquence SPI** : 10 MHz  
- **Data size** : 8 bits (conformément au protocole SPI et au MCP23S17)  
- **NSS Signal Type** : gestion en **software** (NSS Software)

#### 2.2.2 Mapping des broches SPI3

Ensuite, il est nécessaire de mapper correctement les signaux SPI3 du STM32 vers le MCP23S17.  
La configuration retenue est la suivante :

- **SPI3_MOSI** (Master Out Slave In)  → **PB5**  
- **SPI3_MISO** (Master In Slave Out) → **PC11**  
- **SPI3_SCK** (Serial Clock)         → **PC10**  
- **SPI3_/CS** (Chip Select)          → **PB7**  
- **/RESET** du MCP23S17              → **PA0**

Cette configuration est visible sur le schéma des broches généré par STM32CubeIDE :

![Mapping des signaux SPI3 sur la NUCLEO-L476RG](assets/image8.jpg)

Ces broches ont ensuite été configurées dans l’onglet **GPIO Settings** de STM32CubeIDE afin de garantir une communication correcte entre le STM32L476 et le MCP23S17 pour le pilotage du VU-mètre.
### 2.2 Tests

Le MCP23S17 pilote deux réseaux de LEDs connectés respectivement sur ses ports **GPIOA** et **GPIOB**.  
Ces LEDs s’activent à l’état bas (logique inversée). Les tests incluent :

- Activation alternée d'une LED sur deux  
- Validation du bon fonctionnement via un chenillard  

Les registres utilisés pour configurer et contrôler les LEDs sont présentés en détail ci-dessous.

---

#### 2.2.1 Test d'une LED sur 2


![Test d’une LED sur deux](assets/video1_gif.gif)

#### 2.2.2 Chenillard


![Test d’une LED](assets/video2_gif.gif)

### 2.3 Driver et intégration avec le Shell

Un driver dédié a été développé pour gérer l’allumage et l’extinction des LED du MCP23S17.  
Ce driver repose sur une structure permettant d’organiser proprement les registres, l’état des LEDs et les fonctions associées.

Le shell série a ensuite été enrichi de commandes permettant de tester ce driver :

- **Commande `a`** : active simultanément l’ensemble des LED du VU-mètre.
- **Commande `l`** : permet de contrôler une LED précise en fournissant deux paramètres :
  - le numéro de la LED à manipuler,
  - l’état souhaité (1 pour l’allumer, 0 pour l’éteindre).

 ![image13](assets/image13.jpg)

Ces commandes permettent de valider rapidement le fonctionnement du driver ainsi que la communication SPI avec le MCP23S17.

<img src="assets/image12.jpg" width="350">

### 3. Le CODEC Audio SGTL5000

### 3.1 Configurations préalables

Le CODEC **SGTL5000** nécessite une interface I²C pour sa configuration.  
Les lignes utilisées pour l’I²C sur la carte NUCLEO-L476RG sont les suivantes :

- **I2C_SCL : PB10**  
- **I2C_SDA : PB11**

Ces deux broches correspondent à l’interface **I2C2** du STM32L476RG.
 ![image14](assets/image14.jpg)
### 3.1.2 Activation de l’I2C2

Le périphérique **I2C2** a été activé dans STM32CubeIDE en laissant la configuration par défaut.  
Cette interface est utilisée pour configurer les registres du CODEC audio SGTL5000.

![Activation de l’I2C2 dans CubeMX](assets/image19.jpg)

### 3.1.3 Configuration du SAI2 

Le bloc **SAI A** a été configuré en mode **Master with Master Clock Out** afin de générer le signal d’horloge MCLK nécessaire au CODEC.  
Le protocole **I2S/PCM** a été activé avec une taille de données de **16 bits** et un fonctionnement en mode **stéréo**.
![Configuration](assets/image21.jpg)

### 3.1.4 Mapping des broches du SAI2

Les broches du périphérique **SAI2** ont été vérifiées et placées conformément à l’énoncé du TP.  
Les signaux sont connectés comme suit :

- **PB12 → SAI2_FS_A**
- **PB13 → SAI2_SCK_A**
- **PB14 → SAI2_MCLK_A**
- **PB15 → SAI2_SD_A**
- **PC12 → SAI2_SD_B**

Cette configuration assure une communication correcte entre le STM32 et le CODEC audio SGTL5000.

![Mapping des broches SAI2](assets/image23.jpg)

 ### 3.1.5 Configuration de l’horloge du SAI2 (PLLSAI1)

Dans l’onglet **Clock Configuration** de STM32CubeIDE, le **PLLSAI1** a été configuré afin d’obtenir une fréquence de **12.235294 MHz** pour le périphérique **SAI2**, conformément à l’énoncé du TP.

Cette fréquence correspond au signal **MCLK (Master Clock)** nécessaire au bon fonctionnement du CODEC audio **SGTL5000**.

![Configuration du PLLSAI1 pour obtenir 12.235294 MHz au SAI2](assets/image15.jpg)


### 3.1.6 Configuration détaillée des blocs SAI A et SAI B

Le bloc **SAI A** a été configuré en mode **Master Transmit** avec génération d’horloge (MCLK), en utilisant le protocole **I2S Standard** et une taille de données de **16 bits**.

![Configuration détaillée du bloc SAI A](assets/image16.jpg)

Le bloc **SAI B** a été configuré en mode **Slave Receive**, synchronisé avec le bloc SAI A, tout en conservant le protocole **I2S Standard** et une taille de données de **16 bits**.

![Configuration détaillée du bloc SAI B](assets/image17.jpg)

### 3.1.7 Activation des interruptions du SAI2

Les interruptions du périphérique **SAI2** ont été activées dans l’onglet **NVIC** de STM32CubeIDE.  
Cela permet au microcontrôleur de gérer les événements liés au transfert des données audio (transmission et réception via DMA).

![Activation des interruptions du SAI2](assets/image20.jpg)

### 3.1.8 Configuration du DMA pour le SAI2

Le **DMA** a été configuré pour les deux blocs du SAI2 afin de permettre le transfert continu des données audio :

- **SAI2_A → DMA1 Channel 6**
- **SAI2_B → DMA1 Channel 7**
- Mode de fonctionnement : **Circulaire**

Cette configuration est nécessaire pour assurer un flux de données audio continu entre le STM32 et le CODEC SGTL5000.

![Configuration DMA pour SAI2](assets/image18.jpg)

### 3.1.9 Activation manuelle de l’horloge MCLK

Avant toute tentative de communication avec le CODEC SGTL5000, l’horloge **MCLK** doit être activée.  
Cette horloge est générée par le bloc **SAI A**.

Pour cela, la ligne suivante a été ajoutée dans la fonction `main()`, juste après l’initialisation des périphériques :

```c
__HAL_SAI_ENABLE(&hsai_BlockA2);
```
### 3.2.1 Vérification du signal MCLK à l’oscilloscope

À l’aide d’un oscilloscope, la présence du signal **MCLK** généré par le bloc **SAI2** a été vérifiée sur la broche **PB14 (SAI2_MCLK_A)**.

La mesure indique une fréquence d’environ **12.3 MHz**, ce qui correspond à la valeur attendue (**12.235294 MHz**).  
Cela confirme que la configuration du **PLLSAI1** et du **SAI2** est correcte.

![Mesure du signal MCLK à l’oscilloscope (≈12.3 MHz)](assets/your_image_name.jpg)









