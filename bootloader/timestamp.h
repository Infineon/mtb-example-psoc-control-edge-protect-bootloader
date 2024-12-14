/********************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include "cy_pdl.h"

#define TIMESTAMP_SOURCE CY_SYSTICK_CLOCK_SOURCE_CLK_LF
#define TIMESTAMP_DIVIDER (CY_SYSCLK_ILO_FREQ / 1000UL)

/*******************************************************************************
* Function Name: log_timestamp_get
****************************************************************************//**
*
* \brief Get current timestamp counter value.
*
* \return Systic counter as timestamp reference.
*/
static inline uint32_t log_timestamp_get(void) {
    return ((0x1000000UL - Cy_SysTick_GetValue()) / TIMESTAMP_DIVIDER);
}

/*******************************************************************************
* Function Name: log_timestamp_reset
****************************************************************************//**
*
* \brief Reset timestamp counter.
*/
static inline void log_timestamp_reset(void) {
    Cy_SysTick_Init(TIMESTAMP_SOURCE, 0xFFFFFFu);
    Cy_SysTick_DisableInterrupt();
}

/*******************************************************************************
* Function Name: log_timestamp_init
****************************************************************************//**
*
* \brief Initializate timestamp counter and SysTick timebase.
*/
static inline void log_timestamp_init(void) {
    log_timestamp_reset(); 
    Cy_SysTick_Clear();
}

/*******************************************************************************
* Function Name: log_timestamp_deinit
****************************************************************************//**
*
* \brief Deinitializate timestamp counter and SysTick timebase.
*/
static inline void log_timestamp_deinit(void) {
    Cy_SysTick_Disable();
    Cy_SysTick_Clear();
}

#endif /* TIMESTAMP_H */
