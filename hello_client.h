/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/** @file
*
* BLE Vendor Specific Device
*
* This file provides definitions and function prototypes for Hello Sensor
* device
*
*/
#ifndef _HELLO_SENSOR_H_
#define _HELLO_SENSOR_H_

#ifndef BTSTACK_VER
 #include "wiced_gki.h"
 #if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
  #include "wiced_bt_app_common.h"
  #include "wiced_bt_app_hal_common.h"
 #endif
#endif
#include "wiced_hal_nvram.h"

#ifndef PACKED
#define PACKED
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* GPIO pins */
#ifdef CYW20706A2
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_BUTTON
#define HELLO_CLIENT_GPIO_SETTINGS                   WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE )
#define HELLO_CLIENT_DEFAULT_STATE                   WICED_GPIO_BUTTON_DEFAULT_STATE
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            WICED_BUTTON_PRESSED_VALUE
#endif

#if defined(CYW20719B0) || defined(CYW20735B0)
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_PIN_BUTTON
#define HELLO_CLIENT_GPIO_SETTINGS                   ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE )
#define HELLO_CLIENT_DEFAULT_STATE                   GPIO_PIN_OUTPUT_LOW
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            WICED_BUTTON_PRESSED_VALUE
#endif

#if defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20819A1) || defined(CYW55572)
#define HELLO_CLIENT_GPIO_BUTTON                     WICED_GPIO_PIN_BUTTON_1
#define HELLO_CLIENT_BUTTON_PRESSED_VALUE            wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1)
#endif

#if defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1) || defined(CYW30739A0) || defined(CYW55572)
#define HELLO_SENSOR_LED_GPIO        (uint32_t)*platform_led[hello_sensor_led_pin].gpio
#endif

#define HCLIENT_APP_TIMEOUT_IN_SECONDS              1       /* Hello Client App Timer Timeout in seconds  */

#define HELLO_CLIENT_MAX_PERIPHERALS                3       /* Hello Client maximum number of peripherals that can be connected */
#define HELLO_CLIENT_MAX_CONNECTIONS                4       /* Hello Client maximum number of connections including central/peripheral */

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

enum
{
    HANDLE_HCLIENT_GATT_SERVICE = 0x1,                         // GATT service handle

    HANDLE_HCLIENT_GAP_SERVICE = 0x14,                         // GAP service handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME,              // device name characteristic handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,          // char value handle

        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE,        // appearance characteristic handle
        HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,    // char value handle

    HANDLE_HELLO_CLIENT_SERVICE =  0x28,                       // Hello Client Service
        HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY,               // notify characteristic handle
        HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,           // characteristic value handle
        HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,         // characteristic client configuration descriptor handle

    HANDLE_HCLIENT_DEV_INFO_SERVICE = 0x40,                    // Device Information Service
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME,         // manufacturer name characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,     // characteristic value handle

        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM,        // model number characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,    // characteristic value handle

        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,        // system ID characteristic handle
        HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,    // characteristic value handle

    HANDLE_HCLIENT_BATTERY_SERVICE = 0x60,                     // battery service handle
        HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL,             // battery level characteristic handle
        HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL          // characteristic value handle
};


/* Peer Info */
typedef struct
{
    uint16_t conn_id;                   // Connection Identifier
    uint8_t  role;                      // central or peripheral in the current connection
    uint8_t  addr_type;                 // peer address type
    uint8_t  transport;                 // peer connected transport
    uint8_t  peer_addr[BD_ADDR_LEN];    // Peer BD Address
} hello_client_peer_info_t;

/* structure to store GATT attributes for read/write operations */
typedef struct
{
    uint16_t    handle;
    uint16_t    attr_len;
    const void *p_attr;
} gatt_attribute_t;

/* Host information to be stored in NVRAM */
typedef struct
{
    BD_ADDR  bdaddr;                                // BD address of the bonded host
    uint16_t characteristic_client_configuration;   // Current value of the client configuration descriptor
} hclient_host_info_t;

/* Hello client application info */
typedef struct
{
    uint32_t                 app_timer_count;                         // App Timer Count
    uint16_t                 conn_id;                                 // Hold the peripheral connection id
    uint8_t                  num_connections;                         // Number of connections
    uint16_t                 central_conn_id;                         // Handle of the central connection
    uint8_t                  battery_level;                           // dummy battery level
    hclient_host_info_t      host_info;                               // NVRAM save area
    hello_client_peer_info_t peer_info[HELLO_CLIENT_MAX_CONNECTIONS]; // Peer Info
} hello_client_app_t;


#if BTSTACK_VER >= 0x03000001
 #define wiced_bt_gatt_send_notification(id, type, len, ptr)                        wiced_bt_gatt_server_send_notification(id, type, len, ptr, NULL)
 #define wiced_bt_gatt_send_indication(id, type, len, ptr)                          wiced_bt_gatt_server_send_indication(id, type, len, ptr, NULL)
#endif

#endif // _HELLO_SENSOR_H_
