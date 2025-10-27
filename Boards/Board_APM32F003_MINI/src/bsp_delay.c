/*!
 * @file        bsp_delay.c
 *
 * @brief       Delay board support package body
 *
 * @version     V1.0.0
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
 *  that it will be useful and instructional for customers to develop
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
  @{
*/

/** @defgroup APM32F003_MINI_Variables
  @{
*/

/* The stack of delay */
__IO uint8_t __delayStack;

/* The count of delay */
__IO int32_t __delayCnt;

/**@} end of group APM32F003_MINI_Variables */

/** @defgroup APM32F003_MINI_Functions
  @{
*/

/*!
 * @brief       Delay Initialization
 *
 * @param       delayType: specifies the time unit of the delay
 *              This parameter can be one of following parameters"
 *              @arg APM_DELAY_SECOND : the unit of time is second
 *              @arg APM_DELAY_MS     : the unit of time is millisecond
 *              @arg APM_DELAY_100US  : the unit of time is 100 microsecond
 *              @arg APM_DELAY_10US   : the unit of time is 10 microsecond
 *              @arg APM_DELAY_US     : the unit of time is microsecond
 *
 * @retval      None
 */
void APM_DelayInit(APM_Delay_typedef delayType)
{
    if (SysTick_Config(RCM_GetMasterClockFreq() / delayType))
    {
        while (1);
    }
}

/*!
 * @brief       Configure Delay count.
 *
 * @param       nms: Specifies the delay to be configured.
 *              This parameter can be one of following parameters:
 *              @arg cnt : Its value between 0x7FFFFFFF to 0x0
 *
 * @retval      None
 */
void APM_Delay(__IO uint32_t cnt)
{
    /* Clear SysTick count value */
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    /* push into delay stack */
    __delayStack += 1;
    __delayCnt = (cnt & 0x7FFFFFFF);
    while (__delayCnt > 0);
    /* stack out */
    if((__delayStack--) == 1)
    {
        SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
    }
}

/**@} end of group APM32F003_MINI_Functions */
/**@} end of group Board_APM32F003_MINI */
/**@} end of group Board */
