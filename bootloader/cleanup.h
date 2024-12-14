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

#ifndef CLEANUP_H
#define CLEANUP_H

#include <stdint.h>

#include "cy_prot.h"
#include "cy_ms_ctl.h"

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define STACKLESS __STATIC_INLINE
#elif defined(__ICCARM__)
#define STACKLESS __stackless __STATIC_INLINE
#elif defined(__GNUC__)
#define STACKLESS __STATIC_INLINE
#endif

#define CLEANUP_HELPER_AREA_SIZE 0x400
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define NON_CLEANUP_SECTION __attribute__ ((section(".cleanup_helper_area")))
#elif defined(__ICCARM__)
#define NON_CLEANUP_SECTION __root
#elif defined(__GNUC__)
#define NON_CLEANUP_SECTION __attribute__ ((section(".cleanup_helper_area")))
#endif

typedef __NO_RETURN void (*reset_handler_t)(void);

typedef struct vect_tbl_start_s {
    uint32_t        stack_pointer;
    reset_handler_t reset_handler;
} vect_tbl_start_t;

/*******************************************************************************
 * Function Name: cleanup_helper
 ********************************************************************************
 * Summary:
 * Cleans ram region
 * This function used inside cleanup_and_boot function
 *
 * Parameters:
 *  l - region start pointer(lower address)
 *  r - region end pointer (higher address)
 *
 * Note:
 *   This function is critical to be "stackless".
 *   Two oncoming indices algorithm is used to prevent compiler optimization
 *     from calling memset function.
 *
 *******************************************************************************/
NON_CLEANUP_SECTION STACKLESS
void cleanup_helper(register uint8_t *l, register uint8_t *r)
{
    register uint8_t v = 0u;

    do {
        *l = v;
        ++l;

        --r;
        *r = v;
    } while (l < r);
}

/*******************************************************************************
 * Function Name: cleanup_and_boot
 ********************************************************************************
 * Summary:
 * This function cleans all ram and boots target app
 *
 * Parameters:
 * p_vect_tbl_start - target app vector table address
 *
 *
 *******************************************************************************/
NON_CLEANUP_SECTION STACKLESS __NO_RETURN
void cleanup_and_boot(register vect_tbl_start_t* p_vect_tbl_start)
{
    /* Init next app vector table */
    MXCM33->CM33_NS_VECTOR_TABLE_BASE = (uint32_t)(void*)p_vect_tbl_start;
    SCB->VTOR = (uint32_t)(void*)p_vect_tbl_start;

    __DSB();

    /* Init next app stack pointer */
    __set_MSP(p_vect_tbl_start->stack_pointer);

    /* Cleanup ram */
    cleanup_helper((uint8_t*)RAM_ORIGIN + CLEANUP_HELPER_AREA_SIZE, (uint8_t*)(RAM_ORIGIN + RAM_SIZE));

#ifdef USE_PROT_CONTEXT_SWITCH
    Cy_Ms_Ctl_SetActivePC(CPUSS_MS_ID_CM33_0, 3);
#endif

    /* Jump to next app */
    p_vect_tbl_start->reset_handler();
}

#endif /* CLEANUP_H */
