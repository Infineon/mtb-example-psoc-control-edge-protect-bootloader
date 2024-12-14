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

#ifndef MCUBOOT_CONFIG_H
#define MCUBOOT_CONFIG_H

/*
 * Template configuration file for MCUboot.
 *
 * When porting MCUboot to a new target, copy it somewhere that your
 * include path can find it as mcuboot_config/mcuboot_config.h, and
 * make adjustments to suit your platform.
 *
 * For examples, see:
 *
 * boot/zephyr/include/mcuboot_config/mcuboot_config.h
 * boot/mynewt/mcuboot_config/include/mcuboot_config/mcuboot_config.h
 */
/* Default maximum number of flash sectors per image slot; change
 * as desirable. */
#ifndef MCUBOOT_MAX_IMG_SECTORS
#define MCUBOOT_MAX_IMG_SECTORS 128U
#endif

/*
 * Signature types
 *
 * You must choose exactly one signature type.
 */

/* Uncomment for RSA signature support */
//#define MCUBOOT_SIGN_RSA

/* Uncomment for ECDSA signatures using curve P-256. */
#define MCUBOOT_SIGN_EC256

// #define MCUBOOT_SIGN_EC

/*
 * Upgrade mode
 *
 * The default is to support A/B image swapping with rollback.  A
 * simpler code path, which only supports overwriting the
 * existing image with the update image, is also available.
 */
#ifdef MCUBOOT_OVERWRITE_ONLY
/* Uncomment to only erase and overwrite those slot 0 sectors needed
 * to install the new image, rather than the entire image slot. */
/* #define MCUBOOT_OVERWRITE_ONLY_FAST */
#else
/* Using SWAP w Scratch by default.
 * Uncomment which is needed. */
#define MCUBOOT_SWAP_USING_SCRATCH  1
/* #define MCUBOOT_SWAP_USING_MOVE     1 */
#define MCUBOOT_SWAP_USING_STATUS   1
#endif

/* Save ENC IV for encryption image */
#define MCUBOOT_SAVE_ENC_IV 1

/*
 * Cryptographic settings
 *
 * You must choose between mbedTLS and Tinycrypt as source of
 * cryptographic primitives. Other cryptographic settings are also
 * available.
 */

/* Uncomment to use ARM's mbedTLS cryptographic primitives */
#define MCUBOOT_USE_MBED_TLS
#define MCUBOOT_USE_PSA_CRYPTO
/* Uncomment to use Tinycrypt's. */
/* #define MCUBOOT_USE_TINYCRYPT */

/*
 * Flash abstraction
 */

/* Uncomment if your flash map API supports flash_area_get_sectors().
 * See the flash APIs for more details. */
// TODO: FWSECURITY-755
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS


/* Use custom interface for SHA256 module */
#define MCUBOOT_SHA256_CUSTOM_INTERFACE

/*
 * Currently there is no configuration option, for this platform,
 * that enables the system specific mcumgr commands in mcuboot
 */
#define MCUBOOT_PERUSER_MGMT_GROUP_ENABLED 0

/*
 * Logging
 */

#define MCUBOOT_HAVE_LOGGING 1
/* Define this to support native mcuboot logging system */
#define CONFIG_MCUBOOT 1

/*
 * Assertions
 */

/* Uncomment if your platform has its own mcuboot_config/mcuboot_assert.h.
 * If so, it must provide an ASSERT macro for use by bootutil. Otherwise,
 * "assert" is used. */
//#define MCUBOOT_HAVE_ASSERT_H

#define MCUBOOT_WATCHDOG_FEED()         \
    do {                                \
        /* TODO: to be implemented */   \
    } while (0)

/* Uncomment these if support of encrypted upgrade image is needed */
#ifdef ENC_IMG
#define MCUBOOT_ENC_IMAGES
#define MCUBOOT_ENCRYPT_EC256
#endif /* ENC_IMG */

/*
 * No direct idle call implemented
 */
#define MCUBOOT_CPU_IDLE() \
    do {                   \
    } while (0)

/*
 * Do not save ENCTLV by default
 */
//#define MCUBOOT_SWAP_SAVE_ENCTLV 1

#endif /* MCUBOOT_CONFIG_H */
