/*******************************************************************************
 * @file    IKS01A2.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    01.01.2019
 *
 * @brief   Interfacing X-NUCLEO-IKS01A2 motion MEMS and environmental
 *          sensor expansion board.
 * @note
 *
@verbatim
Copyright (C) Almohandes.org, 2019

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/

/* Define to prevent recursive inclusion */
#ifndef __INC_IKS01A2_H_
#define __INC_IKS01A2_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f4xx.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @addtogroup IKS01A2
 * @{
 */

/**
 * @defgroup IKS01A2_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup IKS01A2_exported_functions
 * @{
 */
void IKS01A2_Init(void);
void IKS01A2_Main(void);
void I2C3_TX_DMA_IRQ_Callback(void);
void I2C3_RX_DMA_IRQ_Callback(void);

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /*__INC_IKS01A2_H_ */

