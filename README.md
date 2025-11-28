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

Ces commandes permettent de valider rapidement le fonctionnement du driver ainsi que la communication SPI avec le MCP23S17.
![image12](assets/image12)



