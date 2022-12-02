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
 * SPP Application for 20XXX devices.
 *
 * SPP application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports single a single SPP connection.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *  SEND_DATA_ON_INTERRUPT   - if defined, the app will send 1Meg of data on application button push
 *  SEND_DATA_ON_TIMEOUT     - if defined, the app will send 4 bytes every second while session is up
 *  LOOPBACK_DATA            - if enabled, the app sends back received data
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to the WICED board.
 * 2. Open the Bluetooth Classic and LE Profile Client Control application and open the port for WICED HCI for the device.
 *    Default baud rate configured in the application is defined by the BSP HCI_UART_DEAULT_BAUD #define,
 *    usually either 3M or 115200 depending on board UART capabilities.
 * 3. Run the BTSpy program to view protocol and application traces.
 *    See "Bluetooth Classic and LE Profile Client Control" and "BTSpy" in chip-specifc readme.txt for more information about these apps.
 * 4. On Windows 10 PCs, right click on the Bluetooth icon in the system tray and
 *    select 'Add a Bluetooth Device'. Find and pair with the spp app. That should create an incoming and an outgoing
 *    COM port on your computer. Right click on the Bluetooth icon in the system tray and
 *    select 'Open Settings', scroll down and select "More Bluetooth options" and then
 *    select the 'COM Ports' tab.
 * 5. Use application such as Term Term to open the outgoing COM port. Opening the port
 *    will create the SPP connection.
 * 6. Type any key on the terminal of the outgoing COM port, the spp application will receive the key.
 * 7. By default, (SEND_DATA_ON_INTERRUPT=1) the application sends 1 MB data to the peer application on every
 *    App button press on the WICED board.
 * 8. If desired, edit the spp.c file to configure the application to send data on a timer to the peer application by
 *    setting SEND_DATA_ON_INTERRUPT=0 and SEND_DATA_ON_TIMEOUT=1*
 *
 * Features demonstrated
 *  - Use of SPP library
 *
 *  Note: This snippet app does not support WICED HCI Control and may use transport only for tracing.
 *  If you route traces to WICED HCI UART, use ClientControl app with baud rate equal to that
 *  set in the wiced_transport_cfg_t structure below (currently set at HCI_UART_DEFAULT_BAUD, either 3 Mbps or 115200
 *  depending on the capabilities of the board used).
 */

#include "ikh_spp.h"
#include "sparcommon.h"
#include "bt_types.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"
#include "wiced_hci.h"
#include "wiced_timer.h"
#include "wiced_transport.h"

#include "wiced_memory.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_rfcomm.h"
#include "cycfg_sdp_db.h"


#include "wiced_hal_puart.h"
#include "wiced_hal_gpio.h"

#define HCI_TRACE_OVER_TRANSPORT            1
//#define LOOPBACK_DATA                     1   // If defined application loops back received data

#define WICED_EIR_BUF_MAX_SIZE              264
#define SPP_NVRAM_ID                        WICED_NVRAM_VSID_START
#define WICED_NVRAM_VSID_START_2            0x02
#define LED_STATUS_NVRAM_ID                 WICED_NVRAM_VSID_START_2

/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER                       1017
#define TRANS_MAX_BUFFERS                   2
#define TRANS_UART_BUFFER_SIZE              1024
#define SPP_MAX_PAYLOAD                     1007

#if SEND_DATA_ON_INTERRUPT
#define APP_TOTAL_DATA_TO_SEND             1000000
#define BUTTON_GPIO                         WICED_P30
int     app_send_offset = 0;
uint8_t app_send_buffer[SPP_MAX_PAYLOAD];
uint32_t time_start = 0;
#endif

#ifdef SEND_DATA_ON_TIMEOUT
wiced_timer_t spp_app_timer;
void app_timeout(TIMER_PARAM_TYPE arg);
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
#define SPP_RFCOMM_SCN               2

#define MAX_TX_RETRY                 30
#define TX_RETRY_TIMEOUT             100 // msec

static void         spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         spp_connection_down_callback(uint16_t handle);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static void         spp_bt_remote_name_callback(wiced_bt_dev_remote_name_result_t *p_remote_name_result);

wiced_bt_spp_reg_t spp_reg =
{
    SPP_RFCOMM_SCN,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    NULL,                               /* SPP connection establishment failed, not used because this app never initiates connection */
    NULL,                               /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};
#if BTSTACK_VER < 0x03000001
wiced_transport_buffer_pool_t*  host_trans_pool;
#endif
uint16_t                        spp_handle = 0;
wiced_timer_t                   app_tx_timer;
uint32_t                        spp_rx_bytes = 0;
uint32_t                        spp_tx_retry_count = 0;

uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
#if BTSTACK_VER < 0x03000001
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
#endif

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#if BTSTACK_VER >= 0x03000001
        .heap_config =
        {
            .data_heap_size = 1024 * 4 + 1500 * 2,
            .hci_trace_heap_size = 1024 * 2,
            .debug_trace_heap_size = 1024,
        },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
#endif
    .p_status_handler    = NULL,
    .p_data_handler      = NULL,
    .p_tx_complete_cback = NULL
};
#endif
/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
 //Set LED Pin -> I2S_DI/PCM_IN(P12) ikh@221128

/* Delay timer for LED blinking */
#define APP_TIMEOUT_IN_MILLI_SECONDS_50                 50       /* Milli Seconds timer */
#define APP_TIMEOUT_IN_MILLI_SECONDS_250                250       /* Milli Seconds timer */
#define APP_TIMEOUT_IN_MILLI_SECONDS_500                500      /* Milli Seconds timer */
#define APP_TIMEOUT_IN_MILLI_SECONDS_1000               1000      /* Milli Seconds timer */

static BYTE prev_blink_status = GPIO_PIN_OUTPUT_LOW; //타입변경 타입통일
static BYTE curr_blink_status = GPIO_PIN_OUTPUT_HIGH;
static uint32_t prev_blink_interval = APP_TIMEOUT_IN_MILLI_SECONDS_250;
static uint32_t curr_blink_interval = 0;

wiced_timer_t seconds_timer;                   /* app seconds timer */
uint32_t wiced_timer_count       = 0;          /* number of seconds elapsed */


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
//gpio
static void seconds_app_timer_cb( uint32_t arg );

//interrupt
static void gpio_interrrupt_handler(void *data, uint8_t port_pin);

//puart
void wiced_hal_puart_reset_puart_interrupt(void);
void wiced_hal_puart_register_interrupt(void (*puart_rx_cbk)(void*));
void puart_rx_interrupt_callback(void* unused);

//spp
static wiced_bt_dev_status_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                  app_write_eir(void);
static int                   app_write_nvram(int nvram_id, int data_len, void *p_data);
static int                   app_read_nvram(int nvram_id, void *p_data, int data_len);

#if SEND_DATA_ON_INTERRUPT
static void                  app_tx_ack_timeout(TIMER_PARAM_TYPE param);
static void                  app_interrupt_handler(void *data, uint8_t port_pin);
#endif
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif

#if defined (CYW20706A2)
//extern BOOL32 wiced_hal_puart_select_uart_pads(UINT8 rxdPin, UINT8 txdPin, UINT8 ctsPin, UINT8 rtsPin);
extern wiced_result_t wiced_bt_app_init( void );
#endif

#ifndef CYW20706A2
extern uint64_t clock_SystemTimeMicroseconds64();
#else
#include "rtc.h"
uint64_t clock_SystemTimeMicroseconds64(void)
{
    tRTC_REAL_TIME_CLOCK rtcClock;
    rtc_getRTCRawClock(&rtcClock);
    // To convert 128 kHz rtc timer to milliseconds divide it by 131: //128 kHz = 128 * 1024 = 131072; to microseconds: 1000000 / 131072 = 7.62939453125 (7.63)
    return rtcClock.rtc64 * 763 / 100;
}
#endif

/*******************************************************************
 * Function Definitions
 ******************************************************************/

void buffer_report(char *msg)
{
     /* !BTSTACK_VER < 0x03000001 */

    wiced_bt_buffer_statistics_t buffer_stats[5];
    wiced_result_t result;

    result = wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

    if (result == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("%s: pool %x %d %d/%d/%d\n", msg,
                buffer_stats[1].pool_id,
                buffer_stats[1].pool_size,
                buffer_stats[1].current_allocated_count,
                buffer_stats[1].max_allocated_count,
                buffer_stats[1].total_count);
        WICED_BT_TRACE("%s: pool %x %d %d/%d/%d\n", msg,
                buffer_stats[2].pool_id,
                buffer_stats[2].pool_size,
                buffer_stats[2].current_allocated_count,
                buffer_stats[2].max_allocated_count,
                buffer_stats[2].total_count);
    }
    else
        WICED_BT_TRACE("buffer_report: wiced_bt_get_buffer_usage failed, returned %d\n", result);
}

/*
 * Entry point to the application. Set device configuration and start Bluetooth
 * stack initialization.  The actual application initialization will happen
 * when stack reports that Bluetooth device is ready
 */
APPLICATION_START()
{
    wiced_result_t result;
    int interupt = 0, timeout = 0, loopback = 0;

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init(&transport_cfg);

#if BTSTACK_VER < 0x03000001
    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);
#endif

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart) if platform has PUART
#ifdef NO_PUART_SUPPORT
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("*************Starting PUART Application**********\n\r");

#if SEND_DATA_ON_INTERRUPT
    interupt = 1;
#endif
#if SEND_DATA_ON_TIMEOUT
    timeout = 1;
#endif
#if LOOPBACK_DATA
    loopback = 1;
#endif

    WICED_BT_TRACE("APP Start, interupt=%d, timeout=%d, loopback=%d\n", interupt, timeout, loopback);

    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
    wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);

}

/*
 * SPP application initialization is executed after Bluetooth stack initialization is completed.
 */
void application_init(void)
{
    wiced_result_t         result;

    uint16_t pin_config, pin_config2;

    /* Initialize wiced app */
    wiced_bt_app_init();

    /* Initialize the RTC block */
    rtc_init();

    /* Initializes the GPIO driver */
    wiced_bt_app_hal_init();

    /* initial the led blink timer */
    wiced_init_timer( &seconds_timer, &seconds_app_timer_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER );
            

    /**
     * This function turns off flow control, and enables Tx
     * and Rx. Echoes the input byte with increment by 1.
     */
    /* BEGIN - puart interrupt */
    wiced_hal_puart_reset_puart_interrupt();
    wiced_hal_puart_register_interrupt(puart_rx_interrupt_callback);

    /* Turn on Tx */
    wiced_hal_puart_enable_tx();
    wiced_hal_puart_print( "Type something! Keystrokes are echoed to the terminal ...\r\n");


    /* Configure LED PIN as input and initial outvalue as high
     * Configure GPIO PIN# as input, pull up and interrupt on rising edge and output value as high
     *  (pin should be configured before registering interrupt handler )
     */

    wiced_hal_gpio_configure_pin( WICED_GPIO_LED, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );
    wiced_hal_gpio_set_pin_output( WICED_GPIO_LED, GPIO_PIN_OUTPUT_LOW);

    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON_TOGGLE, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON_ONOFF, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );



    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON_TOGGLE, gpio_interrrupt_handler, NULL );
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON_ONOFF, gpio_interrrupt_handler, NULL );

    WICED_BT_TRACE( "The initial value of gpio_test_led FUNC is set to LOW!\n" );

    /* Get the pin configuration set above */
    pin_config = wiced_hal_gpio_get_pin_config( WICED_GPIO_BUTTON_TOGGLE );
    WICED_BT_TRACE( "Pin config of P%d is %d\n", WICED_GPIO_BUTTON_TOGGLE, pin_config );
    pin_config2 = wiced_hal_gpio_get_pin_config( WICED_GPIO_BUTTON_ONOFF );
    WICED_BT_TRACE( "Pin config of P%d is %d\n", WICED_GPIO_BUTTON_ONOFF, pin_config2 );

    app_write_eir();

#if defined (CYW20706A2)
    // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
    // stack pools configured in the wiced_bt_cfg.c
    wiced_bt_rfcomm_init(MAX_TX_BUFFER, 1);
#endif

    // Initialize SPP library
    wiced_bt_spp_startup(&spp_reg);

    /* create SDP records */
    wiced_bt_sdp_db_init((uint8_t *)sdp_database, sdp_database_len);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    // This application will always configure device connectable and discoverable
    wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
        wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval,
        wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window);

    wiced_bt_dev_set_connectability(BTM_CONNECTABLE,
        wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval,
        wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window);


#ifdef HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the Bluetooth stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(app_trace_callback);
#endif

#if SEND_DATA_ON_INTERRUPT
	wiced_hal_gpio_configure_pin(WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
	wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, app_interrupt_handler, NULL);
    // init timer that we will use for the rx data flow control.
    wiced_init_timer(&app_tx_timer, app_tx_ack_timeout, 0, WICED_MILLI_SECONDS_TIMER);
#endif

#if SEND_DATA_ON_TIMEOUT
    /* Starting the app timers, seconds timer and the ms timer  */
    wiced_init_timer(&spp_app_timer, app_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer(&spp_app_timer, 1);
#endif
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t               dev_status;
    wiced_bt_dev_pairing_info_t*        p_pairing_info;
    wiced_bt_dev_encryption_status_t*   p_encryption_status;
    int                                 bytes_written, bytes_read;
    wiced_bt_power_mgmt_notification_t* p_power_mgmt_notification;

    WICED_BT_TRACE("app_management_callback %d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        application_init();

        if(app_read_led_onoff_nvram(LED_STATUS_NVRAM_ID, &curr_blink_status, sizeof(curr_blink_status)) != 0)
        {
            wiced_hal_gpio_set_pin_output( WICED_GPIO_LED, curr_blink_status);
        }
        else
        {
            WICED_BT_TRACE("LED Key retrieval failure\n");
        }
        if(app_read_led_toggle_nvram(LED_STATUS_NVRAM_ID, &curr_blink_interval, sizeof(curr_blink_interval)) != 0)
        {

            wiced_start_timer( &seconds_timer, curr_blink_interval );
        }
        else
        {
            WICED_BT_TRACE("LED Key retrieval failure\n");
        }

        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* This application always confirms peer's attempt to pair */
        wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        /* get the remote name to show in the log */
        result = wiced_bt_dev_get_remote_name(p_event_data->user_confirmation_request.bd_addr, spp_bt_remote_name_callback);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* This application supports only Just Works pairing */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
        WICED_BT_TRACE("Pairing Complete: %d\n", p_pairing_info->br_edr.status);
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
        app_write_nvram(SPP_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */
        if (app_read_nvram(SPP_NVRAM_ID, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd (%B) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
        break;

    default:
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}

/*
 *  Prepare extended inquiry response data.  Current version publishes device name and 16bit
 *  SPP service.
 */
void app_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint16_t eir_length;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE("hci_control_write_eir %x\n", pBuf);

    if (!pBuf)
    {
        WICED_BT_TRACE("app_write_eir %x\n", pBuf);
        return;
    }

    p = pBuf;

    length = strlen((char *)wiced_bt_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;        // EIR type full name
    memcpy(p, wiced_bt_cfg_settings.device_name, length);
    p += length;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_SERIAL_PORT & 0xff;
    *p++ = (UUID_SERVCLASS_SERIAL_PORT >> 8) & 0xff;

    *p++ = 0;                                       // end of EIR Data is 0

    eir_length = (uint16_t) (p - pBuf);

    // print EIR data
    WICED_BT_TRACE_ARRAY(pBuf, MIN(p-pBuf, 100), "EIR :");
    wiced_bt_dev_write_eir(pBuf, eir_length);

    return;
}

/*
 * The function invoked on timeout of app seconds timer.
 */
#if SEND_DATA_ON_TIMEOUT
void app_timeout(TIMER_PARAM_TYPE arg)
{
    static uint32_t timer_count = 0;
    timer_count++;
    wiced_bool_t ret;
    WICED_BT_TRACE("app_timeout: %d, handle %d \n", timer_count, spp_handle);
    if (spp_handle != 0)
    {
        ret = wiced_bt_spp_send_session_data(spp_handle, (uint8_t *)&timer_count, sizeof(uint32_t));
        if (ret != WICED_TRUE)
            WICED_BT_TRACE("wiced_bt_spp_send_session_data failed, ret = %d\n", ret);
    }
}
#endif

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    WICED_BT_TRACE("%s handle:%d address:%B\n", __FUNCTION__, handle, bda);
    spp_handle = handle;
    spp_rx_bytes = 0;
}

/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
    WICED_BT_TRACE("%s handle:%d rx_bytes:%d\n", __FUNCTION__, handle, spp_rx_bytes);
    spp_handle = 0;
#if defined(SEND_DATA_ON_INTERRUPT) && (SEND_DATA_ON_INTERRUPT==1)
    app_send_offset = 0;
    spp_tx_retry_count = 0;

    if(wiced_is_timer_in_use(&app_tx_timer))
    wiced_stop_timer(&app_tx_timer);
#endif
}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
//    int i;
//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);
//    buffer_report("spp_rx_data_callback");

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));

    spp_rx_bytes += data_len;

    WICED_BT_TRACE("%s handle:%d len:%d %02x-%02x, total rx %d\n", __FUNCTION__, handle, data_len, p_data[0], p_data[data_len - 1], spp_rx_bytes);

    wiced_hal_puart_synchronous_write(p_data, data_len); //ikh@221129 => spp > com(puart)

#if LOOPBACK_DATA
    return wiced_bt_spp_send_session_data(handle, p_data, data_len);
#else
    return WICED_TRUE;
#endif
}

/*
 * spp_bt_remote_name_callback
 */
static void spp_bt_remote_name_callback(wiced_bt_dev_remote_name_result_t *p_remote_name_result)
{
    WICED_BT_TRACE("Pairing with: BdAddr:%B Status:%d Len:%d Name:%s\n",
            p_remote_name_result->bd_addr, p_remote_name_result->status,
            p_remote_name_result->length, p_remote_name_result->remote_bd_name);
}

 

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int app_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}
int app_write_led_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int app_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result);
    }
    return (read_bytes);
}
int app_read_led_toggle_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(curr_blink_interval))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(curr_blink_interval), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(curr_blink_status), read_bytes, result);
    }
    return (read_bytes);
}
int app_read_led_onoff_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(curr_blink_interval))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(curr_blink_interval), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(curr_blink_interval), read_bytes, result);
    }
    return (read_bytes);
}
#if SEND_DATA_ON_INTERRUPT
/*
 * Test function which sends as much data as possible.
 */
void app_send_data(void)
{
    int i;
    wiced_bool_t ret;

    while ((spp_handle != 0) && (app_send_offset != APP_TOTAL_DATA_TO_SEND))
    {
        int bytes_to_send = app_send_offset + SPP_MAX_PAYLOAD < APP_TOTAL_DATA_TO_SEND ? SPP_MAX_PAYLOAD : APP_TOTAL_DATA_TO_SEND - app_send_offset;
        ret = wiced_bt_spp_can_send_more_data(spp_handle);
        if(!ret)
        {
            // buffer_report(" app_send_data can't send");
            // WICED_BT_TRACE(" ! return from wiced_bt_spp_can_send_more_data\n");
            break;
        }
        for (i = 0; i < bytes_to_send; i++)
        {
            app_send_buffer[i] = app_send_offset + i;
        }
        ret = wiced_bt_spp_send_session_data(spp_handle, app_send_buffer, bytes_to_send);
        if(ret != WICED_TRUE)
        {
            // WICED_BT_TRACE(" ! return from wiced_bt_spp_send_session_data\n");
            break;
        }
        app_send_offset += bytes_to_send;
        spp_tx_retry_count = 0;
    }
    // Check if we were able to send everything
    if (app_send_offset < APP_TOTAL_DATA_TO_SEND)
    {
        if(spp_tx_retry_count >= MAX_TX_RETRY)
        {
		WICED_BT_TRACE("Reached max tx retries! Terminating transfer!\n");
		WICED_BT_TRACE("Make sure peer device is providing us credits\n");
		app_send_offset = 0;
        }
        else
        {
            WICED_BT_TRACE("wiced_start_timer app_tx_timer %d\n", app_send_offset);
		wiced_start_timer(&app_tx_timer, TX_RETRY_TIMEOUT);
		spp_tx_retry_count++;
        }
    }
    else
    {
        uint32_t time_tx = clock_SystemTimeMicroseconds64() / 1000 - time_start;
        WICED_BT_TRACE("sent %d in %dmsec (%dKbps)\n", APP_TOTAL_DATA_TO_SEND, time_tx, APP_TOTAL_DATA_TO_SEND * 8 / time_tx);
        app_send_offset = 0;
    }
}

/*
 * Test function which start sending data.
 */
void app_interrupt_handler(void *data, uint8_t port_pin)
{
    WICED_BT_TRACE("gpio_interrupt_handler pin:%d send_offset:%d\n", port_pin, app_send_offset);
    time_start = clock_SystemTimeMicroseconds64() / 1000;

     /* Get the status of interrupt on P# */
    if (wiced_hal_gpio_get_pin_interrupt_status(BUTTON_GPIO))
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(BUTTON_GPIO);
    }
    // If we are already sending data, do nothing
    if (app_send_offset != 0)
        return;

    app_send_data();
}

/*
 * The timeout function is periodically called while we are sending big amount of data
 */
void app_tx_ack_timeout(TIMER_PARAM_TYPE param)
{
    app_send_data();
}
#endif

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
	return; //ikh@221130 - no log msg
#if BTSTACK_VER >= 0x03000001
    wiced_transport_send_hci_trace( type, p_data, length );
#else
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
#endif
}
#endif

/**
 @brief  Interrupt routine called when PUART has data to be read
 @param  unused    Not used
 @return        void
 */
void puart_rx_interrupt_callback(void* unused)
{
    /* There can be at most 16 bytes in the HW FIFO.*/
    uint8_t  readbyte=0;

    /* Drain rx and send to tx. We don't want to wait on tx and assume it won't be overrun (because RTS/CTS) */
    while (wiced_hal_puart_rx_fifo_not_empty() && wiced_hal_puart_read(&readbyte))
    {
    	readbyte;
        // this will wait for tx fifo empty before queueing byte
        wiced_hal_puart_synchronous_write(&readbyte,1);
        wiced_bt_spp_send_session_data(spp_handle, &readbyte, 1);
    }
    #if !PUART_RTS_CTS_FLOW
    wiced_hal_puart_reset_puart_interrupt( );
    #endif
}

#if PUART_RTS_CTS_FLOW
void puart_allow_interrupt()
{
    wiced_hal_puart_reset_puart_interrupt( );
}
#endif

/**
 * ikh@221130 - gpio func
 */
void gpio_interrrupt_handler(void *data, uint8_t port_pin) //ikh@221128
{

    WICED_BT_TRACE("app_management_callback %d\n", port_pin);

    switch (port_pin)
    {
        case WICED_GPIO_BUTTON_TOGGLE/* constant-expression */:
            /* Get the status of interrupt on P# */
            if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_BUTTON_TOGGLE ) )
            {

                /* Clear the gpio interrupt */
                wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_BUTTON_TOGGLE );

            }
            /* stop the led blink timer */
            wiced_stop_timer( &seconds_timer );

            
            /* toggle the blink time interval */
            curr_blink_interval = (APP_TIMEOUT_IN_MILLI_SECONDS_250 == prev_blink_interval)?APP_TIMEOUT_IN_MILLI_SECONDS_1000:APP_TIMEOUT_IN_MILLI_SECONDS_250;

            //wiced_start_timer( &seconds_timer, APP_TIMEOUT_IN_MILLI_SECONDS_50 );
            wiced_start_timer( &seconds_timer, curr_blink_interval );
            
            app_write_led_nvram(LED_STATUS_NVRAM_ID, sizeof(curr_blink_interval), &curr_blink_interval);
            
            WICED_BT_TRACE("gpio_interrupt_handler : %d\n\r", curr_blink_interval);

            //app_write_led_nvram(LED_TOGGLE_NVRAM_ID, sizeof(curr_blink_interval), &curr_blink_interval);
            /* update the previous blink interval */
            prev_blink_interval = curr_blink_interval;
            

            break;

        case WICED_GPIO_BUTTON_ONOFF:

             /* Get the status of interrupt on P# */
            if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_BUTTON_ONOFF) )
            {

                /* Clear the gpio interrupt */
                wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_BUTTON_ONOFF );

            }
            /* stop the led blink timer */
            wiced_stop_timer( &seconds_timer );

            /* toggle the blink time interval */
            curr_blink_status = (GPIO_PIN_OUTPUT_LOW == prev_blink_status)?GPIO_PIN_OUTPUT_HIGH:GPIO_PIN_OUTPUT_LOW;

            wiced_hal_gpio_set_pin_output( WICED_GPIO_LED, curr_blink_status);
            app_write_led_nvram(LED_STATUS_NVRAM_ID, sizeof(curr_blink_status), &curr_blink_status);
 
            WICED_BT_TRACE("gpio_interrupt_handler : %d\n\r", curr_blink_status);

            prev_blink_status = curr_blink_status;


            break;

        default:
            break;
    }

}

/* The function invoked on timeout of app seconds timer. */
void seconds_app_timer_cb( uint32_t arg )
{
    wiced_timer_count++;
    WICED_BT_TRACE( "seconds periodic timer count: %d s\n", wiced_timer_count );

    if(wiced_timer_count & 1)
    {
        wiced_hal_gpio_set_pin_output( WICED_GPIO_LED, GPIO_PIN_OUTPUT_LOW);
    }
    else
    {
        wiced_hal_gpio_set_pin_output( WICED_GPIO_LED, GPIO_PIN_OUTPUT_HIGH);
    }
}
