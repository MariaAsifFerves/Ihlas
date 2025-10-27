/*!
 * @file        Board_APM32F003_MINI.c
 *
 * @brief       This file provides firmware functions to manage Leds and key buttons
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

/* Includes */
#include "Board.h"

/** @addtogroup Board
  @{
*/

/** @addtogroup Board_APM32F003_MINI
  * @brief APM32F003 MINI Board Drivers
  @{
*/

/** @defgroup APM32F003_MINI_Variables
  @{
*/

/* APM MINI Board's LED object table */
const APM_MINI_LED_T APM_MINI_LED[APM_MINI_LEDn] =
{
    /* LED2 */
    {APM_MINI_LED2_GPIO_PORT, APM_MINI_LED2_GPIO_PIN},
    /* LED3 */
    {APM_MINI_LED3_GPIO_PORT, APM_MINI_LED3_GPIO_PIN},
} ;

/* APM MINI Board's button object table */
const APM_MINI_BUTTON_T APM_MINI_BUTTON[APM_MINI_BUTTONn] =
{
    /* Key1 */
    {APM_MINI_KEY1_GPIO_PORT, APM_MINI_KEY1_GPIO_PIN, APM_MINI_KEY1_EINT_PORT}
} ;

/* APM MINI Board's COM object table */
const APM_MINI_COM_T APM_MINI_COM[APM_MINI_COMn] =
{
    /* COM1 */
    {
        APM_MINI_COM1_TX_PORT, APM_MINI_COM1_RX_PORT, APM_MINI_COM1_TX_PIN, APM_MINI_COM1_RX_PIN,
        APM_MINI_COM1_TX_IRQn, APM_MINI_COM1_RX_IRQn, APM_MINI_COM1_CLK, APM_MINI_COM1
    },
    /* COM2 */
    {
        APM_MINI_COM2_TX_PORT, APM_MINI_COM2_RX_PORT, APM_MINI_COM2_TX_PIN, APM_MINI_COM2_RX_PIN,
        APM_MINI_COM2_TX_IRQn, APM_MINI_COM2_RX_IRQn, APM_MINI_COM2_CLK, APM_MINI_COM2
    },
    /* COM3 */
    {
        APM_MINI_COM3_TX_PORT, APM_MINI_COM3_RX_PORT, APM_MINI_COM3_TX_PIN, APM_MINI_COM3_RX_PIN,
        APM_MINI_COM3_TX_IRQn, APM_MINI_COM3_RX_IRQn, APM_MINI_COM3_CLK, APM_MINI_COM3
    }
} ;

/**@} end of group APM32F003_MINI_Variables */

/** @defgroup APM32F003_MINI_Functions
  @{
*/

/*!
 * @brief       Led initialization
 *
 * @param       led:    Specifies the Led to be initialized
 *                      This parameter can be one of following parameters
 *                      @arg LED2
 *                      @arg LED3
 *
 * @retval      None
 */
void APM_MINI_LEDInit(APM_MINI_Led_TypeDef led)
{
    GPIO_Config_T gpioConfig;

    gpioConfig.mode = GPIO_MODE_OUT_OD;
    gpioConfig.speed = GPIO_SPEED_10MHz;
    gpioConfig.intEn = GPIO_EINT_DISABLE;
    gpioConfig.pin = APM_MINI_LED[led].pin;

    GPIO_Config(APM_MINI_LED[led].port, &gpioConfig);
}

/*!
 * @brief       Turns the specified led on
 *
 * @param       led:    Specifies the Led to be set on
 *
 * @retval      None
 */
void APM_MINI_LEDOn(APM_MINI_Led_TypeDef led)
{
    GPIO_ClearBit(APM_MINI_LED[led].port, APM_MINI_LED[led].pin);
}

/*!
 * @brief       Turns the specified led off
 *
 * @param       led:    Specifies the Led to be set off
 *
 * @retval      None
 */
void APM_MINI_LEDOff(APM_MINI_Led_TypeDef led)
{
    GPIO_SetBit(APM_MINI_LED[led].port, APM_MINI_LED[led].pin);
}

/*!
 * @brief       Toggles the specified LED
 *
 * @param       led:    Specifies the Led to be toggled
 *
 * @retval      None
 */
void APM_MINI_LEDToggle(APM_MINI_Led_TypeDef led)
{
    GPIO_Toggle(APM_MINI_LED[led].port, APM_MINI_LED[led].pin);
}

/*!
 * @brief       Push button initialization
 *
 * @param       key:        Specifies the Key button to be configured
 *
 * @param       buttonMode: Enable or Disable external interrupt.
 *                          It can be one of following parameters
 *                          @arg BUTTON_MODE_GPIO
 *                          @arg BUTTON_MODE_EINT
 *
 * @retval      None
 */
void APM_MINI_PBInit(APM_MINI_Button_TypeDef key, APM_MINI_ButtonMode_TypeDef buttonMode)
{
    GPIO_Config_T gpioConfig;

    gpioConfig.mode = GPIO_MODE_IN_PU;
    gpioConfig.speed = GPIO_SPEED_10MHz;
    gpioConfig.pin = APM_MINI_BUTTON[key].pin;

    if(buttonMode)
    {
        gpioConfig.intEn = GPIO_EINT_ENABLE;
        EINT_Config(APM_MINI_BUTTON[key].eintPort, EINT_TRIGGER_FALLING);
        NVIC_EnableIRQRequest(EINTD_IRQn, 0X0f);
    }
    else
    {
        gpioConfig.intEn = GPIO_EINT_DISABLE;
    }

    GPIO_Config(APM_MINI_BUTTON[key].port, &gpioConfig);
}

/*!
 * @brief       Read the specified key button state
 *
 * @param       key:    Specifies the Key button to be read
 *
 * @retval      The Key GPIO pin value
 */
uint8_t APM_MINI_ReadPBState(APM_MINI_Button_TypeDef key)
{
    return(GPIO_ReadInputBit(APM_MINI_BUTTON[key].port, APM_MINI_BUTTON[key].pin));
}

/*!
 * @brief       COM initialization
 *
 * @param       COM: Specifies the COM port to be configured.
 *              This parameter can be one of following parameters:
 *              @arg COM1
 *              @arg COM2
 *              @arg COM3
 *
 * @param       usartConfigStruct: USART Configuration
 *
 * @retval      None
 */
void APM_MINI_COMInit(APM_MINI_COM_TypeDef COM, USART_Config_T* usartConfigStruct)
{
    GPIO_Config_T gpioConfig;

    /* Enable COM1 or COM2 clock */
    RCM_EnableAPBPeriphClock(APM_MINI_COM[COM].clk);

    /* Configure USART Tx as push-pull */
    gpioConfig.speed = GPIO_SPEED_10MHz;
    gpioConfig.intEn = GPIO_EINT_DISABLE;

    gpioConfig.mode = GPIO_MODE_OUT_PP;
    gpioConfig.pin = APM_MINI_COM[COM].TXpin;
    GPIO_Config(APM_MINI_COM[COM].TXport, &gpioConfig);

    /* Configure USART Rx as input floating */
    gpioConfig.mode = GPIO_MODE_IN_FLOATING;
    gpioConfig.pin = APM_MINI_COM[COM].RXpin;
    GPIO_Config(APM_MINI_COM[COM].RXport, &gpioConfig);

    USART_Config(APM_MINI_COM[COM].usart, usartConfigStruct);

    /* Enable USART_Interrupt_RXBNEIE */
    // USART_EnableInterrupt(APM_MINI_COM[COM].usart, USART_INT_RX);
    // NVIC_EnableIRQRequest(APM_MINI_COM[COM]., 2);

    /* Enable USART */
    USART_Enable(APM_MINI_COM[COM].usart);
}

/**@} end of group APM32F003_MINI_Functions */
/**@} end of group Board_APM32F003_MINI */
/**@} end of group Board */
