/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* LE Vendor Specific Client Device
*
* The Hello Client application is designed to connect and access services
* of the Hello Sensor device.  Hello Client can connect up to three
* Hello Sensor Device's.  Because handles of the all attributes of
* the Hello Sensor are well known, Hello Client does not perform GATT
* discovery, but uses them directly.  In addition to that Hello Client
* allows another central to connect, so the device will behave as a peripheral
* in one Bluetooth piconet and a central in another.  To accomplish that
* application can do both advertisements and scan.  Hello Client assumes
* that Hello Sensor advertises a special UUID and connects to the device
* which publishes it.
*
* Features demonstrated
*  - Registration with LE stack for various events
*  - Connection to a central and a peripheral
*  - As a central processing notifications from the server and
*    sending notifications to the client
*  - As a peripheral processing writes from the client and sending writes
*    to the server
*
* To demonstrate the app, work through the following steps.
* 1. Plug the AIROC eval board into your computer
* 2. Build and download the application (to the AIROC board)
* 3. Connect from some client application (for example LightBlue on iOS)
* 4. From the client application register for notifications
* 5. Make sure that your peripheral device (hello_sensor) is up and advertising
* 6. Push a button on the tag board for 6 seconds.  That will start
*    connection process.
* 7. Push a button on the hello_sensor to deliver notification through
*    hello_client device up to the client
* 8. Repeat the steps 5 to 7 for connecting to multiple hello_sensor
*     device's
*
*/
#include <string.h>
#include "sparcommon.h"
#include "bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_utils.h"
#include "hello_uuid.h"
#include "hello_client.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_transport.h"
#include "wiced_hal_gpio.h"

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_platform.h"
#include "wiced_bt_stack.h"

#include "wiced_hal_puart.h"
#include "wiced_timer.h"
#if !defined(CYW20706A2) && !defined(CYW43022C1) // Remove CYW43022C1 when USE_DESIGN_MODUS is enabled.
#include "cycfg_pins.h"
#endif

/******************************************************************************
 *  Constants
 ******************************************************************************/

/* UUID value of the Hello Client Service */
#define UUID_HELLO_CLIENT_SERVICE   0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc, 0xfa, 0x44, 0x54, 0x7c, 0x0d, 0x90, 0x03, 0xdc

/* UUID value of the Hello Client Data Characteristic */
#define UUID_HELLO_CLIENT_DATA      0xc5, 0x42, 0x45, 0x3b, 0xd0, 0x74, 0x5b, 0x81, 0xf6, 0x4a, 0x26, 0x8f, 0xa5, 0xcf, 0x7a, 0xb7

#define PAIRING_BUTTON_ALWAYS_ON 0
#define PAIRING_BUTTON_TOGGLE    1

/******************************************************************************
 *  Type  Definitions
 ******************************************************************************/

/******************************************************************************
 *  Imported Variables Declrations
 ******************************************************************************/
extern const wiced_transport_cfg_t  transport_cfg;
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************************************
 *  Golbal Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Client application.  Hello Client
 * can connect to hello sensor, and also provides service for
 * somebody to access.  The database defines services, characteristics and
 * descriptors supported by the application.  Each attribute in the database
 * has a handle, (characteristic has two, one for characteristic itself,
 * another for the value).  The handles are used by the peer to access
 * attributes, and can be used locally by application, for example to retrieve
 * data written by the peer.  Definition of characteristics and descriptors
 * has GATT Properties (read, write, notify...) but also has permissions which
 * identify if peer application is allowed to read or write into it.
 * Handles do not need to be sequential, but need to be in order.
 */
const uint8_t hello_client_gatt_database[]=
{
    // Handle 0x01: GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Handle 0x14: GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_GAP_SERVICE, UUID_SERVICE_GAP ),

        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,
             UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value.
        // List of approved appearances is available at bluetooth.org.  Current
        // value is set to 0x200 - Generic Tag
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
             UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x28: Hello Client Service.
    // This is the main proprietary service of Hello Client application.  It has
    // a single characteristic which allows peer to write to and can be configured
    // to send indications to the peer.  Note that UUID of the vendor specific
    // service is 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.
    // _UUID128 version of the macro should be used.
    PRIMARY_SERVICE_UUID128( HANDLE_HELLO_CLIENT_SERVICE, UUID_HELLO_CLIENT_SERVICE ),

        // Handle 0x29: characteristic Hello Notification, handle 0x2a characteristic value
        // we support both notification and indication.  Peer need to allow notifications
        // or indications by writing in the Characteristic Client Configuration Descriptor
        // (see handle 2b below).  Note that UUID of the vendor specific characteristic is
        // 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
        // of the macro should be used.
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,
             UUID_HELLO_CLIENT_DATA, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE |
             GATTDB_CHAR_PROP_WRITE_NO_RESPONSE | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
             GATTDB_PERM_READABLE  | GATTDB_PERM_WRITE_CMD  | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_VARIABLE_LENGTH ),

            // Handle 0x2b: Characteristic Client Configuration Descriptor.
            // This is standard GATT characteristic descriptor.  2 byte value 0 means that
            // message to the client is disabled.  Peer can write value 1 or 2 to enable
            // notifications or indications respectively.  Not _WRITABLE in the macro.  This
            // means that attribute can be written by the peer.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,
                 UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_AUTH_WRITABLE),

    // Handle 0x40: Device Info service
    // Device Information service helps peer to identify manufacture or vendor
    // of the device.  It is required for some types of the devices (for example HID,
    // and medical, and optional for others.  There are a bunch of characteristics
    // available, out of which Hello Sensor implements 3.
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

        // Handle 0x41: characteristic Manufacturer Name, handle 0x42 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
             UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        // Handle 0x43: characteristic Model Number, handle 0x4 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
             UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        // Handle 0x45: characteristic System ID, handle 0x46 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
             UUID_CHARACTERISTIC_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x60: Battery service
    // This is an optional service which allows peer to read current battery level.
    PRIMARY_SERVICE_UUID16( HANDLE_HCLIENT_BATTERY_SERVICE, UUID_SERVICE_BATTERY ),

        // Handle 0x61: characteristic Battery Level, handle 0x62 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL,
             UUID_CHARACTERISTIC_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),
};

size_t hello_client_gatt_database_size = sizeof(hello_client_gatt_database);

/* Holds the hello client app info */
hello_client_app_t g_hello_client;

char    hello_client_local_name[]           = "Hello Client";
uint8_t hello_client_appearance[2]          = { BIT16_TO_8( APPEARANCE_GENERIC_TAG ) };
uint8_t hello_client_notify_value[]         = "Hello Client";
char    hello_client_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
char    hello_client_char_model_num_value[] = { '4', '3', '2', '1', 0, 0, 0, 0 };
uint8_t hello_client_char_system_id_value[] = { 0xef, 0x48, 0xa2, 0x32, 0x17, 0xc6, 0xa6, 0xbc };

size_t hello_client_notify_value_size = sizeof(hello_client_notify_value);


/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
const gatt_attribute_t hello_client_gattdb_attributes[] =
{
    { HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( hello_client_local_name),          hello_client_local_name },
    { HANDLE_HCLIENT_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof( hello_client_appearance),          hello_client_appearance },
    { HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC,          2,                                         &g_hello_client.host_info.characteristic_client_configuration },
    { HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,        sizeof(hello_client_notify_value),         hello_client_notify_value },
    { HANDLE_HCLIENT_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                         &g_hello_client.battery_level },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(hello_client_char_mfr_name_value),  hello_client_char_mfr_name_value },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(hello_client_char_model_num_value), hello_client_char_model_num_value },
    { HANDLE_HCLIENT_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(hello_client_char_system_id_value), hello_client_char_system_id_value },
};

/* Hello service UUID  */
const uint8_t hello_service[16] = {UUID_HELLO_SERVICE};

/* Variable to indicate if the scan has to be started.
 * Set to 1 if the user pushes and holds the button for more than 5 seconds */
uint8_t start_scan = 0;

#ifdef CYW43012C0
/* Application buffer pool. */
wiced_bt_buffer_pool_t*        p_hello_client_buffer_pool;
#endif

wiced_timer_t hello_client_ms_timer;

/******************************************************************************
 *  Imported Function Declartions
 ******************************************************************************/
extern void                     hello_client_gatt_enable_notification ( void );
extern wiced_bt_gatt_status_t   hello_client_gatt_init(void);
extern wiced_result_t           app_stack_init(void);
extern void                     hello_client_interrupt_handler(void* user_data, uint8_t value );
extern wiced_bt_gatt_status_t   hello_client_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
extern void                     hello_client_process_data_from_peripheral( int len, uint8_t *data );

/******************************************************************************
 *  Function Declrations
 ******************************************************************************/
hello_client_peer_info_t *      hello_client_get_peer_information( uint16_t conn_id );
wiced_bool_t                    hello_client_is_device_bonded( wiced_bt_device_address_t bd_address );
const gatt_attribute_t*         hello_client_get_attribute(uint16_t handle);
wiced_result_t                  hello_client_management_cback( wiced_bt_management_evt_t event,  wiced_bt_management_evt_data_t *p_event_data );
int                             hello_client_get_num_peripherals(void);
void                            hello_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
wiced_bt_gatt_status_t          hello_client_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );
void                            hello_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type );

static void                     hello_client_app_init( void );
static void                     hello_client_set_advertisement_data( void );
static void                     hello_client_app_timer( TIMER_PARAM_TYPE arg );
static void                     hello_client_smp_bond_result( BD_ADDR bda, uint8_t result );
static void                     hello_client_encryption_changed( wiced_result_t result, uint8_t* p_bd_addr );
static void                     hello_client_remove_peer_info( uint16_t conn_id );
static wiced_bool_t             hello_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             hello_client_read_link_keys( wiced_bt_device_link_keys_t *p_keys);
static void                     hello_client_load_keys_to_addr_resolution_db( void );
static int                      hello_client_is_central( BD_ADDR bda );

/******************************************************************************
 *  Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that Bluetooth device is ready.
 */
APPLICATION_START( )
{

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef NO_PUART_SUPPORT
    // if a board does not have PUART support, route to WICED UART by default, see below
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else
#if defined(CYW43022C1)
    // 43022 transport clock rate is 24Mhz for baud rates <= 1.5 Mbit/sec, and 48Mhz for baud rates > 1.5 Mbit/sec.
    // HCI UART and Debug UART both use the Transport clock, so if the HCI UART rate is <= 1.5 Mbps, please do not set the Debug UART > 1.5 Mbps.
    // The default Debug UART baud rate is 115200, and default HCI UART baud rate is 3Mbps
    debug_uart_set_baudrate(115200);

    // CYW943022M2BTBGA doesn't have GPIO pin for PUART.
    // CYW943022M2BTBGA Debug UART Tx (WICED_GPIO_02) is connected to UART2_RX (ARD_D1) on PUART Level Translators of CYW9BTM2BASE1.
    // We need to set to WICED_ROUTE_DEBUG_TO_DBG_UART to see traces on PUART of CYW9BTM2BASE1.
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_DBG_UART );
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    app_stack_init();
}


/*
 * hello client device and link management callbacks
 */
wiced_result_t hello_client_management_cback( wiced_bt_management_evt_t event,  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t               result = WICED_BT_SUCCESS;
    wiced_result_t               add_db_result;

    WICED_BT_TRACE("==>hello_client_management_cback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_client_app_init( );
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            {
                wiced_bt_dev_ble_pairing_info_t * p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

                WICED_BT_TRACE( "Pairing Complete: %d\n", p_info->reason );

                hello_client_smp_bond_result( p_event_data->pairing_complete.bd_addr, p_info->reason );
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            {
                wiced_bt_dev_encryption_status_t * p_status =
                    &p_event_data->encryption_status;

                WICED_BT_TRACE( "encryption status: bd ( %B ) res %d\n",
                        p_status->bd_addr,
                        p_status->result);

                hello_client_encryption_changed( p_status->result ,
                        p_status->bd_addr );
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Use the default security */
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            hello_client_save_link_keys( &p_event_data->paired_device_link_keys_update );
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if ( hello_client_read_link_keys( &p_event_data->paired_device_link_keys_request ) )
            {
                WICED_BT_TRACE( "Key retrieval success\n" );
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE( "Key retrieval failure\n" );
            }
            break;


        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
            /* (sample app does not store keys to NVRAM).  New local identity keys will be generated */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE( "Advertisement State Change: %d\n", p_event_data->ble_advert_state_changed);
            if ( p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF )
            {
                if ( g_hello_client.central_conn_id == 0 )
                {
                    // Start the advertisement to enable peripheral connection
                    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
                    WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
                }
            }
            break;

        default:
            break;
    }

    return result;
}

/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
void hello_client_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
 #ifdef NEW_DYNAMIC_MEMORY_INCLUDED
    wiced_transport_send_hci_trace( type, p_data, length );
#else
    wiced_transport_send_hci_trace( NULL, type, length, p_data );
#endif
}
#endif

/*
 * AIROC Bluetooth Init Complete.  This function is called when device initialization
 * has been completed.  Perform the App Initializations & Callback Registrations
 */
void hello_client_app_init( void )
{
    int                    index;
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    WICED_BT_TRACE( "hello_client_app_init\n" );

    memset( &g_hello_client, 0, sizeof( g_hello_client ) );

#ifdef CYW43012C0
    p_hello_client_buffer_pool = wiced_bt_create_pool( 64, 5 );
    if (p_hello_client_buffer_pool == NULL)
        WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");
#endif // CYW43012C0

#ifdef CYW20706A2
    /* initialize common Bluetooth application logic */
    wiced_bt_app_init( );
#endif

#if defined(CYW20706A2)
    wiced_hal_gpio_register_pin_for_interrupt( HELLO_CLIENT_GPIO_BUTTON, hello_client_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( HELLO_CLIENT_GPIO_BUTTON, HELLO_CLIENT_GPIO_SETTINGS , HELLO_CLIENT_DEFAULT_STATE );
#else
#ifdef CYW43022C1
    // Set WICED_BT_GPIO_04 (J4, pin SDA) as User button (J12, pin 1) connected to
    wiced_hal_gpio_select_function(WICED_GPIO_PIN_BUTTON_1, WICED_GPIO);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON_1, GPIO_INPUT_ENABLE | WICED_GPIO_EN_INT_BOTH_EDGE, 1);
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON_1, hello_client_interrupt_handler, NULL);
#else
#ifndef CYW43012C0
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hello_client_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#endif
#endif
#endif
    // reset connection information
    g_hello_client.num_connections = 0;
    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        g_hello_client.peer_info[index].conn_id = 0;
    }

    /* Register with stack to receive GATT related events */
    gatt_status = wiced_bt_gatt_register( hello_client_gatt_callback );

    WICED_BT_TRACE( "wiced_bt_gatt_register status %d \n", gatt_status );

    /*  GATT DB Initialization */
    gatt_status =  hello_client_gatt_init();

    WICED_BT_TRACE( "wiced_bt_gatt_db_init %d \n", gatt_status );

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hello_client_hci_trace_cback );
#endif

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Load the address resolution DB with the keys stored in the NVRAM */
    hello_client_load_keys_to_addr_resolution_db();

    /* Set the advertising data and make the device discoverable */
    hello_client_set_advertisement_data( );

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    /* Init and starting the app timer  */
    wiced_init_timer(&hello_client_ms_timer, hello_client_app_timer, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
    wiced_start_timer( &hello_client_ms_timer, HCLIENT_APP_TIMEOUT_IN_MSECONDS );
    UNUSED_VARIABLE(result);
    UNUSED_VARIABLE(gatt_status);
}

/* This function will be called when connection goes down */
wiced_bt_gatt_status_t hello_client_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_result_t status;

    WICED_BT_TRACE( "hello_client_connection_down %d <%B>\n", g_hello_client.num_connections, p_conn_status->bd_addr );

    /* Check if the device is there in the peer info table */
    if ( ( g_hello_client.num_connections ) && hello_client_get_peer_information( p_conn_status->conn_id ) )
    {
        // Decrement the number of  connections
        g_hello_client.num_connections--;
    }

    if ( p_conn_status->link_role == HCI_ROLE_PERIPHERAL )
    {
        //Resetting the connection handle to the central
        g_hello_client.central_conn_id = 0;
    }

    //Remove the peer info
    hello_client_remove_peer_info( p_conn_status->conn_id );

     /*  Start the inquiry to search for other available peripherals */
    if ( g_hello_client.num_connections < HELLO_CLIENT_MAX_CONNECTIONS )
    {
        uint8_t num_peripherals = hello_client_get_num_peripherals( );

        if ( g_hello_client.central_conn_id == 0 )
        {
            // Start the advertisement to enable peripheral connection
            status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
            WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", status );
        }
    }
    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Configure data to be sent in the advertisement packets
 */
void hello_client_set_advertisement_data( void )
{
    wiced_result_t result;
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t tx_power_level = 0; //0 db

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_TX_POWER;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &tx_power_level;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen(hello_client_local_name);;
    adv_elem[num_elem].p_data       = ( uint8_t* )hello_client_local_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
    WICED_BT_TRACE(" [%s] wiced_bt_ble_set_raw_advertisement_data status %d \n", __FUNCTION__, result);
    UNUSED_VARIABLE(result);
}


int hello_client_get_num_peripherals(void)
{
    int num_peripherals = g_hello_client.num_connections;

    if( g_hello_client.central_conn_id )
    {
        num_peripherals = g_hello_client.num_connections - 1 ;
    }

    return num_peripherals;
}

/* The function invoked on timeout of app seconds timer. */
void hello_client_app_timer( TIMER_PARAM_TYPE arg )
{
    wiced_result_t               status;

    g_hello_client.app_timer_count++;
    //WICED_BT_TRACE( "hello_client_app_timer timer_count: %d\n", g_hello_client.app_timer_count );

    if( start_scan && wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
    {
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
        WICED_BT_TRACE( "wiced_bt_ble_scan: %d\n", status );
    }
    UNUSED_VARIABLE(status);
}

/*
 * Process SMP bonding result.  If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */
void hello_client_smp_bond_result( BD_ADDR bda, uint8_t result )
{
    wiced_result_t status;

    /*  Start the inquiry to search for other available peripherals */
    if ( g_hello_client.num_connections < HELLO_CLIENT_MAX_CONNECTIONS )
    {
        int num_peripherals = hello_client_get_num_peripherals();

        if ( g_hello_client.central_conn_id == 0 )
        {
            // Start the advertisement to enable peripheral connection
            status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
            WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", status );
        }

        if ( ( num_peripherals < HELLO_CLIENT_MAX_PERIPHERALS ) && ( start_scan ) )
        {
            if( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
            {
                status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
                WICED_BT_TRACE( "hello_client_smp_bond_result wiced_bt_ble_scan: %d\n", status );
            }
        }
    }
    UNUSED_VARIABLE(status);
}

/*
 * Process notification from the stack that encryption has been set.
 * If connected client is registered for notification or indication,
 * it is a good time to send it out
 */
void hello_client_encryption_changed( wiced_result_t result, uint8_t* p_bd_addr )
{
    WICED_BT_TRACE( "hello_client_encryption_changed %d", result );

    /* Bonding success */
    if( result == WICED_BT_SUCCESS )
    {
        // When we are connected as a central, we need to enable notifications from the peripheral
        // This needs to be done only once, because client configuration descriptor value
        // should be persistent across connections with bonded devices.
        if ( hello_client_is_central ( p_bd_addr) )
        {
            hello_client_gatt_enable_notification( );
        }
    }
}

/*
 * This function handles the scan results
 */
void hello_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t         status;
    wiced_bool_t           ret_status;
    uint8_t                length;
    uint8_t *              p_data;

    if ( p_scan_result )
    {
        // Advertisement data from hello_server should have Advertisement type SERVICE_UUID_128
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE, &length );

        // Check if  the hello service uuid is there in the advertisement
        if ( ( p_data == NULL ) || ( length != LEN_UUID_128 ) || ( memcmp( p_data, hello_service, LEN_UUID_128 ) != 0 ) )
        {
            // wrong device
            return;
        }

        WICED_BT_TRACE(" Found Device : %B \n", p_scan_result->remote_bd_addr );
        start_scan = 0;
        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, hello_client_scan_result_cback );

        WICED_BT_TRACE( "scan off status %d\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );

        WICED_BT_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
    }
    else
    {
        WICED_BT_TRACE( "Scan completed:\n" );
    }
    UNUSED_VARIABLE(ret_status);
    UNUSED_VARIABLE(status);
}

/*
 * This function adds the peer information to the table
 */
void hello_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == 0 )
        {
            g_hello_client.peer_info[index].conn_id         = conn_id;
            g_hello_client.peer_info[index].role            = role;
            g_hello_client.peer_info[index].transport       = transport;
            g_hello_client.peer_info[index].addr_type       = address_type;
            memcpy( g_hello_client.peer_info[index].peer_addr, p_bd_addr, BD_ADDR_LEN );
            break;
        }
    }
}

/*
 * This function removes the peer information from the table
 */
void hello_client_remove_peer_info( uint16_t conn_id )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == conn_id )
        {
            g_hello_client.peer_info[index].conn_id = 0;
        }
    }
}

/*
 * This function gets the peer address from  the table
 */
hello_client_peer_info_t * hello_client_get_peer_information( uint16_t conn_id )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id == conn_id )
        {
            return &g_hello_client.peer_info[index];
        }
    }
    return NULL;
}

/*
 * Find out if specific device is connected as a central
 */
static int hello_client_is_central( BD_ADDR bda )
{
    int index;

    for ( index = 0; index < HELLO_CLIENT_MAX_CONNECTIONS; index++ )
    {
        if ( g_hello_client.peer_info[index].conn_id != 0 )
        {
            if ( memcmp( g_hello_client.peer_info[index].peer_addr, bda, BD_ADDR_LEN) == 0 )
            {
                return ( g_hello_client.peer_info[index].role == HCI_ROLE_CENTRAL );
            }
        }
    }
    return FALSE;
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t hello_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    id = 0, i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE( "Read NVRAM at:%d bytes:%d result:%d\n", i, bytes_read, result );

        // if failed to read NVRAM, there is nothing saved at that location
        if ( ( result != WICED_SUCCESS ) || ( bytes_read != sizeof( temp_keys ) ) )
        {
            id = i;
            break;
        }
        else
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved, reuse the ID
                id = i;
                break;
            }
        }
    }
    if ( id == 0 )
    {
        // all NVRAM locations are already occupied.  Cann't save anything.
        WICED_BT_TRACE( "Failed to save NVRAM\n" );
        return WICED_FALSE;
    }
    WICED_BT_TRACE( "writing to id:%d\n", id );
    bytes_written = wiced_hal_write_nvram( id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t *)p_keys, &result );
    WICED_BT_TRACE( "Saved %d bytes at id:%d %d\n", bytes_written, id );
    UNUSED_VARIABLE(bytes_written);
    return WICED_TRUE;
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t hello_client_read_link_keys( wiced_bt_device_link_keys_t *p_keys)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved
                memcpy( &p_keys->key_data, &temp_keys.key_data, sizeof( temp_keys.key_data ) );
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
    return WICED_FALSE;
}

void hello_client_load_keys_to_addr_resolution_db( void )
{
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( keys ), (uint8_t *)&keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
#ifdef CYW20706A2
            result = wiced_bt_dev_add_device_to_address_resolution_db( &keys, keys.key_data.ble_addr_type );
#else
            result = wiced_bt_dev_add_device_to_address_resolution_db( &keys );
#endif

            WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
}

/*
 * Find attribute based on attribute handle
 */
const gatt_attribute_t * hello_client_get_attribute(uint16_t handle)
{
    const gatt_attribute_t * puAttributes = hello_client_gattdb_attributes;
    int   i;

    for ( i = 0; i < sizeof( hello_client_gattdb_attributes ) / sizeof( hello_client_gattdb_attributes[0] ); i++, puAttributes++ )
    {
        if ( puAttributes->handle == handle )
        {
            return puAttributes;
        }
    }
    WICED_BT_TRACE( "Attr handle:0x%x not found\n", handle );
    return NULL;
}

/* Check for device entry exists in NVRAM list */
wiced_bool_t hello_client_is_device_bonded( wiced_bt_device_address_t bd_address )
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = WICED_NVRAM_VSID_START; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, bd_address, BD_ADDR_LEN ) == 0 )
            {
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    UNUSED_VARIABLE(bytes_read);
    return WICED_FALSE;
}
