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

#ifndef MCUBOOT_LOGGING_H
#define MCUBOOT_LOGGING_H

#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include "timestamp.h"

static inline void print_msg(char const *format, ...)
{
    va_list args;
    va_start(args, format);

    (void)printf("[%us"
                ".%ums]",
                (unsigned)(log_timestamp_get()/1000U),
                (unsigned)(log_timestamp_get()%1000U));

    (void)vprintf(format, args);

    va_end(args);
}

#define MCUBOOT_LOG_LEVEL_OFF      0
#define MCUBOOT_LOG_LEVEL_ERROR    1
#define MCUBOOT_LOG_LEVEL_WARNING  2
#define MCUBOOT_LOG_LEVEL_INFO     3
#define MCUBOOT_LOG_LEVEL_DEBUG    4

/*
 * The compiled log level determines the maximum level that can be
 * printed.  Messages at or below this level can be printed, provided
 * they are also enabled through the Rust logging system, such as by
 * setting RUST_LOG to bootsim::api=info.
 */
#ifndef MCUBOOT_LOG_LEVEL
#define MCUBOOT_LOG_LEVEL MCUBOOT_LOG_LEVEL_INFO
#endif

#ifdef __BOOTSIM__
int sim_log_enabled(int level);
#else
static inline int sim_log_enabled(int level) {
    (void)level;
    return 1;
}
#endif


#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_ERROR
#define MCUBOOT_LOG_ERR(_fmt, ...)                           \
    do {                                                     \
        if (sim_log_enabled(MCUBOOT_LOG_LEVEL_ERROR) != 0) { \
            print_msg("[ERR] " _fmt "\n\r", ##__VA_ARGS__);  \
        }                                                    \
    } while ((bool)0)
#else
#define MCUBOOT_LOG_ERR(...) IGNORE(__VA_ARGS__)
#endif

#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_WARNING
#define MCUBOOT_LOG_WRN(_fmt, ...)                             \
    do {                                                       \
        if (sim_log_enabled(MCUBOOT_LOG_LEVEL_WARNING) != 0) { \
            print_msg("[WRN] " _fmt "\n\r", ##__VA_ARGS__);    \
        }                                                      \
    } while ((bool)0)
#else
#define MCUBOOT_LOG_WRN(...) IGNORE(__VA_ARGS__)
#endif

#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_INFO
#define MCUBOOT_LOG_INF(_fmt, ...)                          \
    do {                                                    \
        if (sim_log_enabled(MCUBOOT_LOG_LEVEL_INFO) != 0) { \
            print_msg("[INF] " _fmt "\n\r", ##__VA_ARGS__); \
        }                                                   \
    } while ((bool)0)
#else
#define MCUBOOT_LOG_INF(...) IGNORE(__VA_ARGS__)
#endif

#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_DEBUG
#define MCUBOOT_LOG_DBG(_fmt, ...)                           \
    do {                                                     \
        if (sim_log_enabled(MCUBOOT_LOG_LEVEL_DEBUG) != 0) { \
            print_msg("[DBG] " _fmt "\n\r", ##__VA_ARGS__);  \
        }                                                    \
    } while ((bool)0)
#else
#define MCUBOOT_LOG_DBG(...) IGNORE(__VA_ARGS__)
#endif

#define MCUBOOT_LOG_MODULE_DECLARE(domain)  /* ignore */
#define MCUBOOT_LOG_MODULE_REGISTER(domain) /* ignore */

#endif /* MCUBOOT_LOGGING_H */
