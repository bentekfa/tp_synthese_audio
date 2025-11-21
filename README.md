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
![image1](image1)


### 1.3 Redirection de la fonction printf

Afin de faciliter l’affichage sur le terminal série, la fonction `printf` a été redirigée vers l’USART2 grâce à la fonction suivante :

```c
int __io_putchar(int chr)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
    return chr;
}
```
### 1.4 Activation de FreeRTOS et test avec messages périodiques

Le système d’exploitation temps réel **FreeRTOS** a été activé en mode **CMSIS V1**.  
Une tâche a ensuite été créée afin d’envoyer périodiquement le message **"Hello"** sur l’USART2.

L’affichage répété de ce message dans le terminal série confirme que :

- Les tâches FreeRTOS sont exécutées correctement  
- L’ordonnanceur fonctionne comme prévu
  
### 1.5 Mise en place du Shell

Un **shell série** a été mis en place afin de permettre une interaction directe avec la carte via le terminal série.

Au démarrage, le message suivant est affiché :

===== Monsieur Shell v0.2 =====

Ce message indique que le shell a été correctement initialisé et qu’il est prêt à recevoir des commandes.

### 1.6 Test des commandes du Shell

Une première commande de test a été envoyée dans le terminal :


Le shell a alors répondu :


Ce résultat confirme que :

- Le shell fonctionne correctement  
- Les commandes sont reconnues et traitées  
- Le système est prêt pour l’ajout de nouvelles commandes  

---

## Conclusion – Partie Démarrage

Cette première partie du TP a permis de :

- Créer et configurer un projet sur la carte **NUCLEO_L476RG**  
- Vérifier la communication série avec l’USART2  
- Configurer et utiliser correctement la fonction `printf`  
- Activer et utiliser **FreeRTOS** en mode CMSIS V1  
- Mettre en place un shell série interactif et fonctionnel  


