/*!
 * @file        bsp_delay.h
 *
 * @brief       Header for bsp_delay.c module
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

/* Define to prevent recursive inclusion */
#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

/* Includes */
#include "apm32f00x.h"

/** @addtogroup Board
  @{
*/

/** @addtogroup Board_APM32F003_MINI
  @{
*/

/** @defgroup APM32F003_MINI_Enumerations
  @{
*/

typedef enum
{
    APM_DELAY_SECOND  = 1,
    APM_DELAY_MS      = 1000,
    APM_DELAY_100US   = 10000,
    APM_DELAY_10US    = 100000
}APM_Delay_typedef;

/**@} end of group APM32F003_MINI_Enumerations */

/** @defgroup APM32F003_MINI_Variables
  @{
*/

/* extern variables */
extern __IO uint8_t __delayStack;
extern __IO int32_t __delayCnt;

/**@} end of group APM32F003_MINI_Variables */

/** @defgroup APM32F003_MINI_Functions
  @{
*/

void APM_DelayInit(APM_Delay_typedef delayType);
void APM_Delay(__IO uint32_t cnt);

/*!
 * @brief       Delay function's interrupt server function
 *
 * @param       None
 *
 * @retval      None
 *
 * @note        This function need to put into SysTick_Handler
 */
__STATIC_INLINE void APM_DelayIsr(void)
{
    __delayCnt--;
}

/**@} end of group APM32F003_MINI_Functions */
/**@} end of group Board_APM32F003_MINI */
/**@} end of group Board */

#endif /*__BSP_DELAY_H */
