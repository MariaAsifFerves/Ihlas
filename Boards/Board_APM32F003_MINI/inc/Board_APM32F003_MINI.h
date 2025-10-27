/*!
 * @file        Board_APM32F003_MINI.h
 *
 * @brief       This file contains definitions for APM32F003_MINIBOARD's Leds, push-buttons hardware resources
 *
 * @version     V1.0.2
 *
 * @date        2023-03-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Define to prevent recursive inclusion */
#ifndef __BOARD_APM32F003_MINI_H
#define __BOARD_APM32F003_MINI_H

/* Includes */
#include "apm32f00x.h"
#include "apm32f00x_eint.h"
#include "apm32f00x_gpio.h"
#include "apm32f00x_misc.h"
#include "apm32f00x_rcm.h"
#include "apm32f00x_usart.h"

/** @addtogroup Board
  @{
*/

/** @addtogroup Board_APM32F003_MINI
  @{
*/

/** @defgroup APM32F003_MINI_Macros
  @{
*/

/** @defgroup APM32F003_MINI_LED
  @{
*/

#define APM_MINI_LEDn                   2

/**
 * @brief LED2
 */
#define APM_MINI_LED2_GPIO_PIN          GPIO_PIN_5
#define APM_MINI_LED2_GPIO_PORT         GPIOB

/**
 * @brief LED3
 */
#define APM_MINI_LED3_GPIO_PIN          GPIO_PIN_4
#define APM_MINI_LED3_GPIO_PORT         GPIOB

/**@} end of group APM32F003_MINI_LED */

/** @defgroup APM32F003_MINI_BUTTON
  @{
*/

#define APM_MINI_BUTTONn                1

/**
 * @brief Key1 push-button
 */
#define APM_MINI_KEY1_GPIO_PIN          GPIO_PIN_6
#define APM_MINI_KEY1_GPIO_PORT         GPIOD
#define APM_MINI_KEY1_EINT_PORT         EINT_PORT_D

/**@} end of group APM32F003_MINI_BUTTON */

/** @defgroup APM32F003_MINI_COM
  @{
*/

#define APM_MINI_COMn                   3

/**
 * @brief Definition for COM port1, connected to USART1
 */
#define APM_MINI_COM1                   USART1
#define APM_MINI_COM1_CLK               RCM_PERIPH_USART1

#define APM_MINI_COM1_TX_PIN            GPIO_PIN_5
#define APM_MINI_COM1_TX_PORT           GPIOD
#define APM_MINI_COM1_TX_IRQn           USART1_TX_IRQn

#define APM_MINI_COM1_RX_PIN            GPIO_PIN_6
#define APM_MINI_COM1_RX_PORT           GPIOD
#define APM_MINI_COM1_RX_IRQn           USART1_RX_IRQn

/**
 * @brief Definition for COM port2, connected to USART2
 */
#define APM_MINI_COM2                   USART2
#define APM_MINI_COM2_CLK               RCM_PERIPH_USART2

#define APM_MINI_COM2_TX_PIN            GPIO_PIN_3
#define APM_MINI_COM2_TX_PORT           GPIOD
#define APM_MINI_COM2_TX_IRQn           USART2_TX_IRQn

#define APM_MINI_COM2_RX_PIN            GPIO_PIN_2
#define APM_MINI_COM2_RX_PORT           GPIOD
#define APM_MINI_COM2_RX_IRQn           USART2_RX_IRQn

/**
 * @brief Definition for COM port3, connected to USART3
 */
#define APM_MINI_COM3                   USART3
#define APM_MINI_COM3_CLK               RCM_PERIPH_USART3

#define APM_MINI_COM3_TX_PIN            GPIO_PIN_4
#define APM_MINI_COM3_TX_PORT           GPIOB
#define APM_MINI_COM3_TX_IRQn           USART3_TX_IRQn

#define APM_MINI_COM3_RX_PIN            GPIO_PIN_5
#define APM_MINI_COM3_RX_PORT           GPIOB
#define APM_MINI_COM3_RX_IRQn           USART3_RX_IRQn

/**@} end of group APM32F003_MINI_COM */
/**@} end of group APM32F003_MINI_Macros */

/** @defgroup APM32F003_MINI_Enumerations
  @{
*/

typedef enum
{
    LED2 = 0,
    LED3 = 1
} APM_MINI_Led_TypeDef;

typedef enum
{
    BUTTON_KEY1 = 0
} APM_MINI_Button_TypeDef;

typedef enum
{
    BUTTON_MODE_GPIO = 0,
    BUTTON_MODE_EINT = 1
} APM_MINI_ButtonMode_TypeDef;

typedef enum
{
    COM1 = 0,
    COM2 = 1,
    COM3 = 2
} APM_MINI_COM_TypeDef;

/**@} end of group APM32F003_MINI_Enumerations */

/** @defgroup APM32F003_MINI_Structures
  @{
*/

/**
 * @brief   APM MINI Board's LED object
 */
typedef struct
{
    GPIO_T *port;       /*!< LED attribute: gpio port */
    GPIO_PIN_T pin;     /*!< LED attribute: gpio pin  */
} APM_MINI_LED_T;

/**
 * @brief   APM MINI Board's BUTTON object
 */
typedef struct
{
    GPIO_T *port;         /*!< Button attribute: gpio port */
    GPIO_PIN_T pin;       /*!< Button attribute: gpio pin  */
    EINT_PORT_T eintPort; /*!< Button attribute: eint port */
} APM_MINI_BUTTON_T;

/**
 * @brief   APM MINI Board's BUTTON object
 */
typedef struct
{
    GPIO_T *TXport;     /*!< COM attribute: TX gpio port */
    GPIO_T *RXport;     /*!< COM attribute: RX gpio port */
    GPIO_PIN_T TXpin;   /*!< COM attribute: TX gpio pin  */
    GPIO_PIN_T RXpin;   /*!< COM attribute: RX gpio pin  */
    IRQn_Type TXIRQn;   /*!< COM attribute: TX IRQn  */
    IRQn_Type RXIRQn;   /*!< COM attribute: RX IRQn  */
    RCM_PERIPH_T clk;   /*!< COM attribute: COM periph clock */
    USART_T* usart;      /*!< COM attribute: COM USART */
} APM_MINI_COM_T;

/**@} end of group APM32F003_MINI_Structures */

/** @defgroup APM32F003_MINI_Functions
  @{
*/

/* LED */
void APM_MINI_LEDInit(APM_MINI_Led_TypeDef led);
void APM_MINI_LEDOn(APM_MINI_Led_TypeDef led);
void APM_MINI_LEDOff(APM_MINI_Led_TypeDef led);
void APM_MINI_LEDToggle(APM_MINI_Led_TypeDef led);

/* Key button */
void APM_MINI_PBInit(APM_MINI_Button_TypeDef key, APM_MINI_ButtonMode_TypeDef buttonMode);
uint8_t APM_MINI_ReadPBState(APM_MINI_Button_TypeDef key);

/* COM */
void APM_MINI_COMInit(APM_MINI_COM_TypeDef COM, USART_Config_T* usartConfigStruct);

/**@} end of group APM32F003_MINI_Functions */
/**@} end of group Board_APM32F003_MINI */
/**@} end of group Board */

#endif /*__BOARD_APM32F003_MINI_H */
