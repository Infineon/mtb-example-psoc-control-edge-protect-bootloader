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

#include "platform_init.h"
#include <stdlib.h>
#include <string.h>
#include "cy_cryptolite_trng.h"
#include "crypto.h"

cy_rslt_t platform_init(void)
{
    return CY_RSLT_SUCCESS;
}

void platform_deinit(void)
{

}

psa_status_t mbedtls_psa_external_get_random(mbedtls_psa_external_random_context_t *context,
                                             uint8_t *output, size_t len, size_t *olen)
{
    int ret = 0;
    *olen = 0;
    /* temporary random data buffer */
    uint32_t random = 0u;

    (void)context;

    if (CY_CRYPTOLITE_SUCCESS != Cy_Cryptolite_Trng_Init(CRYPTOLITE, NULL)) {
        return PSA_ERROR_GENERIC_ERROR;
    }
    if (CY_CRYPTOLITE_SUCCESS != Cy_Cryptolite_Trng_Enable(CRYPTOLITE)) {
        return PSA_ERROR_GENERIC_ERROR;
    }
    /* Get Random byte */
    while ((*olen < len) && (ret == 0)) {
        if (Cy_Cryptolite_Trng_ReadData(CRYPTOLITE, &random) != CY_CRYPTOLITE_SUCCESS) {
            return PSA_ERROR_GENERIC_ERROR;
        } else {
            for (uint8_t i = 0; (i < 4) && (*olen < len); i++) {
                *output++ = ((uint8_t *)&random)[i];
                *olen += 1;
            }
        }
    }
    random = 0uL;

    (void)Cy_Cryptolite_Trng_Disable(CRYPTOLITE);
    (void)Cy_Cryptolite_Trng_DeInit(CRYPTOLITE);

    return (ret);
}
