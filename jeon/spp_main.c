/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor
 *  Corporation. All rights reserved. This software, including source code, documentation and  related
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit
 * products. Any reproduction, modification, translation, compilation,  or representation of this
 * Software except as specified above is prohibited without the express written permission of
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to
 * the Software without notice. Cypress does not assume any liability arising out of the application
 * or use of the Software or any product or circuit  described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or failure of the
 * Cypress product may reasonably be expected to result  in significant property damage, injury
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * SPP Application for 2070X devices.
 *
 * SPP application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports single a single SPP connection.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *  SEND_DATA_ON_INTERRUPT   - if defined, the app will send 1Meg of data on application button push
 *  SEND_DATA_ON_TIMEOUT     - if enabled, the app will send 4 bytes every second while session is up
 *  LOOPBACK_DATA            - if enabled, the app sends back received data
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Smart Ready (2070x) evaluation board into your computer
 * 2. Configure application using compile flags defined before
 * 3. Add a Bluetooth Device on Windows.  That should create an incoming and outgoing COM ports
 * 4. Use standard terminal emulation application to open the outgoing COM port and send/receive data
 *
 * Features demonstrated
 *  - Use of SPP library
 *
 *  Note: This snippet app does not support WICED HCI Control and may use transport only for tracing
 *  If you route traces to WICED HCI UART, use any ClientControl which support 4mbps.
 */

#include "sparcommon.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"

#include "wiced_timer.h"
#include "wiced_hal_platform.h"
#include "wiced_memory.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_wdog.h"

#include "spp_debug.h"
#include "spp_uart.h"
#include "spp_main.h"
#include "spp_private.h"
#include "spp_connect.h"
#include "spp_gpio.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_gpio.h"

#include "wiced_bt_rfcomm.h"
#include "iap.h"
#include "wiced_bt_iap2.h"

#include "hiddcfa.h"

#include "wiced_bt_l2c.h"
#include "m_led.h"
#ifdef BLE_ENABLE
#include "ble.h"
#endif

#ifdef DEBUG_MAIN
#define MAIN_DEBUG           DEBUG
#define MAIN_DEBUG_ARRAY     DEBUG_ARRAY
#define MAIN_DEBUG_CRIT      DEBUG_CRIT
#else
#define MAIN_DEBUG         
#define MAIN_DEBUG_ARRAY     
#define MAIN_DEBUG_CRIT     
#endif


#if 1   // Uniquest
static void         spp_transport_flow_control_timeout(uint32_t param);
#endif


#ifdef SEND_DATA_ON_TIMEOUT
void app_timeout(uint32_t count);
#endif


uint8_t reconnect_cnt = 0;
/*****************************************************************************
**  Structures
*****************************************************************************/
#define SDP_ATTR_CLASS_ID_128(uuid)                                      \
    SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(17), \
        ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES), uuid

#define SDP_ATTR_UUID128(uuid)          ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES), uuid

extern BOOLEAN btsnd_hcic_write_simple_pairing_mode (UINT8 mode);// spp disable 
extern BOOLEAN btsnd_hcic_rmt_name_req (BD_ADDR bd_addr,
                                                UINT8 page_scan_rep_mode,
                                                UINT8 page_scan_mode,
                                                UINT16 clock_offset);


extern BOOLEAN btsnd_hcic_set_host_flow_ctrl(UINT8 value); 
extern BOOLEAN btsnd_hcic_write_encr_mode (UINT8 mode);                  /* Write encryption mode */
extern BOOLEAN btsnd_hcic_write_auth_enable(UINT8 flag);                         /* Read Authentication Enable */
extern BOOLEAN btsnd_hcic_write_def_policy_set(UINT16 settings);




static void	spp_connection_fail_callback(void);
static void 	spp_connection_not_found_callback(void);
static void 	spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void 	spp_connection_down_callback(uint16_t handle);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static wiced_bt_dev_status_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static  void spp_inquiry_callback(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data);
static int                   app_write_nvram(int nvram_id, int data_len, void *p_data);
static int                   app_read_nvram(int nvram_id, void *p_data, int data_len);
static void uart_tx_timer_fun(void);
static void  link_key_updata_time_Fun(void);


static void                  app_interrupt_handler(void *data, uint8_t port_pin);


#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif




wiced_bt_spp_reg_t spp_reg =
{
    SPP_RFCOMM_SCN,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    spp_connection_fail_callback,                               /* SPP connection establishment failed, not used because this app never initiates connection */
    spp_connection_not_found_callback,                               /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};


wiced_transport_buffer_pool_t*  host_trans_pool;
wiced_timer_t                   app_tx_timer;
static appSppData SppAppData;


const uint8_t app_sdp_db[] = // Define SDP database
{
	//Spp Service
	SDP_ATTR_SEQUENCE_2(237),
	SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes
	SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 bytes
	SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
	SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SPP_RFCOMM_SCN),                 // 17 bytes
	SDP_ATTR_BROWSE_LIST,                                               // 8
	SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
	SDP_ATTR_SERVICE_NAME(10),                                          // 15
	'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',


	// iAP2 Service
	SDP_ATTR_SEQUENCE_1(93),
	SDP_ATTR_RECORD_HANDLE(0x10002),                                    //8
	SDP_ATTR_CLASS_ID_128(IAP2_ACCESSORY_UUID),                         //22
	SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(31),   // 36
	SDP_ATTR_SEQUENCE_1(3),
	SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
	SDP_ATTR_SEQUENCE_1(5),
	SDP_ATTR_UUID16(UUID_PROTOCOL_RFCOMM),
	SDP_ATTR_VALUE_UINT1(IAP_RFCOMM_SCN),
	SDP_ATTR_SEQUENCE_1(17),
	SDP_ATTR_UUID128(IAP2_ACCESSORY_UUID),
	SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(22), // 27
	SDP_ATTR_SEQUENCE_1(20),
	SDP_ATTR_UUID128(IAP2_ACCESSORY_UUID),
	SDP_ATTR_VALUE_UINT2(0x0100),

    // Device ID service
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
    SDP_ATTR_RECORD_HANDLE(0x10002),                            	        // 8 byte
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),  	                // 8
    SDP_ATTR_PROTOCOL_DESC_LIST(1),                         	            // 18
    SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                	    // 6
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            	// 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0401),                        		// 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    	// 6
    SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     	// 5
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) 	// 6
};

// Length of the SDP database
const uint16_t app_sdp_db_len = sizeof(app_sdp_db);

uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    { TRANS_UART_BUFFER_SIZE, 1},
    NULL,
    NULL,
    NULL
};




/*******************************************************************
 * Function Definitions
 ******************************************************************/
appSppData* getSppAppData(void)
{
	return &SppAppData;
}
/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
APPLICATION_START()
{
    wiced_result_t result;

    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(1024/*TRANS_UART_BUFFER_SIZE*/, TRANS_MAX_BUFFERS);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)

#if defined WICED_BT_TRACE_ENABLE 
  //  wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
  // wiced_hal_puart_select_uart_pads( UART_RX, UART_TX, 0, 0);
  // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_HCI_UART);
  wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
  
#endif
	//wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_HCI_UART);
	//  wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);


    MAIN_DEBUG("APP Start\r\n");


    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
	
	wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * AMS application initialization is executed after BT stack initialization is completed.
 */
void application_init(void)
{

	wiced_bt_gatt_status_t gatt_status;
	wiced_result_t         result;
	uint8_t auto_connect;

	/* Initialize wiced app */
	wiced_bt_app_init();  //platform_led_init delete

	

	getSppAppData()->uart_tx_size = 1024;
	

	button_init();
	m_led_init();
	//wiced_hal_gpio_configure_pin( GPIO_INIT_LED,( GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),GPIO_PIN_OUTPUT_LOW);

#ifdef SPP_ENABLE
	

	// Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
	// stack pools configured in the wiced_bt_cfg.c
	wiced_bt_rfcomm_init(MAX_TX_BUFFER, 1);  //rfcomm frame size,

	// Initialize SPP library
	wiced_bt_spp_startup(&spp_reg);



	wiced_init_timer(&getSppAppData()->inquiry_timer, &inquiry_timer_fun, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
	wiced_init_timer(&getSppAppData()->pairing_timer, &pairing_timer_fun, 0, WICED_MILLI_SECONDS_TIMER);
	wiced_init_timer(&getSppAppData()->reconnection_timer, &reconnection_timer_fun, 0, WICED_MILLI_SECONDS_TIMER);
	
	
	
#endif

#ifdef IAP_ENABLE
	//iap initialize
	iap_init();
#endif

	/*uart init*/
	initUart();

	wiced_init_timer(&getSppAppData()->link_key_update_timer, link_key_updata_time_Fun, 0, WICED_MILLI_SECONDS_TIMER );
#ifdef HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the BT stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(app_trace_callback);
#endif

    /* create SDP records */
    wiced_bt_sdp_db_init((uint8_t *)app_sdp_db, sizeof(app_sdp_db));



	

#if 0
	wiced_init_timer(&getSppAppData()->uart_tx_timer, &uart_tx_timer_fun, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
#endif

	wiced_init_timer(&getSppAppData()->rx_flow_on_retry_timer, &Rx_Flow_On_Retry, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);

#ifdef SPP_ENABLE

	getSppAppData()->activeDevice =noActive;
	/*Device setting*/
    if (bt_read_nvram(NVRAM_ID_DEVICE_NAME_LEN,&getSppAppData()->local_device_name_len, 1) != 0) 
    {
		bt_read_nvram(NVRAM_ID_DEVICE_NAME, getSppAppData()->local_device_name,
		getSppAppData()->local_device_name_len);
    }
    else
    {
        getSppAppData()->local_device_name_len = strlen(wiced_bt_cfg_settings.device_name);
        memcpy(getSppAppData()->local_device_name,
                wiced_bt_cfg_settings.device_name,
                getSppAppData()->local_device_name_len);
    }
	
    wiced_bt_dev_set_local_name(getSppAppData()->local_device_name);

    MAIN_DEBUG("DEVICE NAME : %s LEN : %d\r\n", getSppAppData()->local_device_name, getSppAppData()->local_device_name_len);

    /*set button mode*/
    getSppAppData()->button_mode = read_button_mode();



	if(!bt_read_nvram(NVRAM_ID_NUMERIC_ENABLE, &getSppAppData()->numeric_status, 1))
	{
		getSppAppData()->numeric_status=numeric_disable/*numeric_enable*/;
	}
	MAIN_DEBUG("Numeric status: %d\r\n", getSppAppData()->numeric_status);

#else
	getSppAppData()->numeric_status=numeric_disable;
#endif






#ifdef BLE_ENABLE

#ifdef SPP_ENABLE
	 if(!bt_read_nvram(NVRAM_ID_LE_STATUS, &getSppAppData()->le_status, 1))
	{
		getSppAppData()->le_status=le_disable/*le_enable  le_disable*/;
	}
	else
	{
		if(getSppAppData()->le_status == le_enable)

#else
		getSppAppData()->le_status=le_enable;
	{

#endif
		{
			ble_init();
		}
	}

	MAIN_DEBUG("LE En/Disable: %d\r\n", getSppAppData()->le_status);
#endif



	/*for SSP Disable*/
	btsnd_hcic_write_encr_mode(WICED_TRUE);
	btsnd_hcic_write_auth_enable(WICED_TRUE);


#if 0
    /*set pincode*/
	if(bt_read_nvram(NVRAM_ID_PINCODE_ENABLE, &getSppAppData()->pincode_enable, 1))
	{
		if(getSppAppData()->pincode_enable==pincode_enable)
		{
			btsnd_hcic_write_simple_pairing_mode(0); // spp disable
		}
	}
	else
	{
		getSppAppData()->pincode_enable=pincode_disable;
	}

#endif

    if (!bt_read_nvram(NVRAM_ID_PINCODE_VALUE, getSppAppData()->pincode, 4))
    {
        uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };
        memcpy(getSppAppData()->pincode, pincode, 4);
    }

	getSppAppData()->pincode_flag=0;
	
    MAIN_DEBUG("pincode enable : %d pincode value  %B\r\n",  getSppAppData()->pincode_enable,getSppAppData()->pincode);

    MAIN_DEBUG("pincode : %B\r\n", getSppAppData()->pincode);

	app_write_eir();
	getSppAppData()->button_mode=read_button_mode();
	//setSppState(sppDevReady);
	

	/*pio connection detect setting*/
	if(bt_read_nvram(NVRAM_ID_PIO_CONNECTION_DETECT,&getSppAppData()->pio_connection_detect_mode,1))
	{	
		MAIN_DEBUG("pio connection detect %d\r\n",getSppAppData()->pio_connection_detect_mode);
		if(getSppAppData()->pio_connection_detect_mode==pio_connection_enable)
		{
			MAIN_DEBUG("pio connection detect enable\r\n");		
		}
	}
	else
	{
		getSppAppData()->pio_connection_detect_mode=pio_connection_disable;
	}


	sendEventToUart(init_completed,1,0);
	
	/*auto connection setting*/
	if(bt_read_nvram(NVRAM_ID_AUTO_CONNECTION,&auto_connect,1))
	{	
		if(auto_connect==auto_connection_enable)
		{
			MAIN_DEBUG("auto connection enable\r\n");
			//last_device_connect();		
		}
	}



	//power_on();

	
	getSppAppData()->power_on_flag=true;
	getSppAppData()->button_flag = (getSppAppData()->button_flag |FLAG_GPIO_POWER_ON);
	wiced_start_timer(&getSppAppData()->button_timer, BUTTON_TIMER_INTERVAL);
		
	

	
/*
	if(!wiced_hal_gpio_get_pin_input_status(GPIO_MODE))
	{
		wiced_result_t result;
		uint8_t i;


		
		wiced_hal_delete_nvram(NVRAM_ID_LAST_DEVICE+i,&result);
		

		for(i=0; i<MAX_BONDED_DEVICE+1;i++)
		{
			wiced_hal_delete_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i-1,&result);
		}
		wiced_start_timer( &getSppAppData()->pairing_timer,2000);
	}
	else
	{
		last_device_connect();		
	}
	*/
	//btsnd_hcic_write_simple_pairing_mode(0); // spp disabl
    //setSppState(sppDevDiscoverable);

	

	MAIN_DEBUG("\r\n application init\r\n");	
}


void wiced_bt_dev_vendor_specific_command_callback(wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params)
{
	MAIN_DEBUG("wiced_bt_dev_vendor_specific_command_callback \r\n");
	MAIN_DEBUG("opcode : %x , len %d parm %d\r\n",p_command_complete_params->opcode,p_command_complete_params->param_len,p_command_complete_params->p_param_buf[0]);
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
    wiced_bt_device_link_keys_t        device_link_key;
	wiced_bt_ble_connection_param_update_t *p_ble_conn_param_update;

#ifdef BLE_ENABLE
	uint8_t *p_keys;
	wiced_bt_ble_advert_mode_t *p_mode;
#endif

    MAIN_DEBUG("bt_management_callback 0x%02x\r\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        application_init();
        MAIN_DEBUG("Free mem:%d\r\n", wiced_memory_get_free_bytes());
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PIN_REQUEST_EVT:
        MAIN_DEBUG("BTM_PIN_REQUEST_EVT : remote address= %B\r\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,/*result*/WICED_BT_SUCCESS, 4, getSppAppData()->pincode);
		getSppAppData()->pincode_flag=1;
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* This application always confirms peer's attempt to pair */
        MAIN_DEBUG("BTM_USER_CONFIRMATION_REQUEST_EVT \r\n");

 		if(getSppAppData()->numeric_status==numeric_enable)
		{
			uart_pairing_user_confirm(p_event_data->user_confirmation_request.numeric_value);
			getSppAppData()->authentication_reject_evet_flag=1;		

		
			//wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
		}
	 	else
	 	{
	 		wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
	 	}
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* This application supports only Just Works pairing */
        MAIN_DEBUG("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\r\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
	if(getSppAppData()->numeric_status==numeric_enable)
	{
       	p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   =BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;// BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
       	p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_YES;//BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
	}
	else
	{
		p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   =BTM_IO_CAPABILITIES_NONE;
		p_event_data->pairing_io_capabilities_br_edr_request.auth_req       =BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;// BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
	}
	break;
    case BTM_PASSKEY_REQUEST_EVT:
		MAIN_DEBUG("BTM_PASSKEY_REQUEST_EVT \r\n");
		uint32_t passkey=0;
		wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS,p_event_data->user_passkey_request.bd_addr, passkey);
		break;
    case BTM_PAIRING_COMPLETE_EVT:
    {
		p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
		MAIN_DEBUG("Pairing Complete: EDR %d  BLE : %d\r\n", p_pairing_info->br_edr.status,p_pairing_info->ble.status);	
		// MAIN_DEBUG("private : %d ,resolve addr : %B, resolve addr type %d\r\n",p_pairing_info->ble.privacy_supported,p_pairing_info->ble.resolved_bd_addr,p_pairing_info->ble.resolved_bd_addr_type);
	if((getSppAppData()->numeric_status==numeric_enable) &(getSppAppData()->pincode_flag==0))
	{
		if(((p_pairing_info->br_edr.status ==0x05) |((p_pairing_info->ble.status==0x05)|(p_pairing_info->ble.status==8110)))&(getSppAppData()->authentication_reject_evet_flag==1))
		{
			MAIN_DEBUG("authentication rejected by remote device \r\n");
			getSppAppData()->authentication_reject_evet_flag=0;	
			uart_authentication_reject();
			if(p_pairing_info->ble.status==8110)
			{
				 LE_Disconnect();
			}
		}
		else if((p_pairing_info->br_edr.status ==0x00) |(p_pairing_info->ble.status==0x00))
		{
			sendEventToUart(pairing_complete,1,0); 	
		}
	}
	
	getSppAppData()->pincode_flag=0;

	result = WICED_BT_SUCCESS_NO_SECURITY;
    }
      break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        MAIN_DEBUG("Encryption Status Event: bd (%B) res %d P_ref_data %B\r\n", p_encryption_status->bd_addr, p_encryption_status->result,p_encryption_status->p_ref_data);
  	break;
	
    case BTM_SECURITY_ABORTED_EVT:
    {
		MAIN_DEBUG(" BTM_SECURITY_ABORTED_EVT : bd (%B) \r\n",p_event_data->security_aborted.bd_addr);
    }
		break;
		
    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
    {
		/* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
		MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT\r\n");

		uint8_t i;
		uint8_t BR_BLE_flag=0;
		for(i=0;i<16;i++)
		{
			MAIN_DEBUG("%02x ",p_event_data->paired_device_link_keys_update.key_data.br_edr_key[i]);
			BR_BLE_flag = BR_BLE_flag | p_event_data->paired_device_link_keys_update.key_data.br_edr_key[i];
		}
		MAIN_DEBUG("\r\n");

		getSppAppData()->link_key_update_timer_data.transport=BR_BLE_flag;
		memcpy(&getSppAppData()->link_key_update_timer_data.link_key_updata,&p_event_data->paired_device_link_keys_update,sizeof(p_event_data->paired_device_link_keys_update));

		getSppAppData()->device_type_flag=WICED_TRUE;

		wiced_start_timer(&getSppAppData()->link_key_update_timer, LINK_KEY_UPDATE_TIMEOUT);
    }
    break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
    {
		MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\r\n");
		
		memcpy(&device_link_key,&p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t));
	       //bt_read_nvram(NVRAM_ID_LAST_DEVICE, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t));

		MAIN_DEBUG("key_ble_typetype %x \r\n",device_link_key.key_data.le_keys_available_mask);
		
		if(device_link_key.key_data.le_keys_available_mask == 0)
		{
			MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT  BR_EDR\r\n");
			if(find_bonded_device(device_link_key.bd_addr, &p_event_data->paired_device_link_keys_request))
			{
				 result = WICED_BT_SUCCESS;
				MAIN_DEBUG("Key success\r\n");
			}
			else
			{
				result = WICED_BT_ERROR;
				MAIN_DEBUG("Key retrieval failure\r\n");
			}
		}
		else
		{
			MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT  BLE\r\n");
			if(find_ble_bonded_device(device_link_key.bd_addr, &p_event_data->paired_device_link_keys_request))
			{
				 result = WICED_BT_SUCCESS;
				MAIN_DEBUG("Key success\r\n");
			}
			else
			{
				result = WICED_BT_ERROR;
				MAIN_DEBUG("Key retrieval failure\r\n");
			}
		}
    }
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        MAIN_DEBUG("Power mgmt status event: bd (%B) status:%d hci_status:%d\r\n", p_power_mgmt_notification->bd_addr, p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
        break;

#ifdef BLE_ENABLE
	case BTM_PASSKEY_NOTIFICATION_EVT:
        MAIN_DEBUG("LE_PassKey Notification. BDA %B, Key %d \r\n",
                p_event_data->user_passkey_notification.bd_addr,
                p_event_data->user_passkey_notification.passkey);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                p_event_data->user_passkey_notification.bd_addr);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        MAIN_DEBUG("LE_BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT \r\n");

	if(getSppAppData()->numeric_status==numeric_enable)
	{
	        p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
	                BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT; //BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
	        p_event_data->pairing_io_capabilities_ble_request.auth_req =
	        BTM_LE_AUTH_REQ_SC_MITM_BOND;//BTM_LE_AUTH_REQ_SC_BOND;//BTM_LE_AUTH_REQ_SC_MITM_BOND;
	}
	else
	{
		 p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
	                BTM_IO_CAPABILITIES_NONE; //BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
	        p_event_data->pairing_io_capabilities_ble_request.auth_req =
	                BTM_LE_AUTH_REQ_BOND; //BTM_LE_AUTH_REQ_SC_BOND;
	}
        p_event_data->pairing_io_capabilities_ble_request.oob_data =
                BTM_OOB_NONE;
      
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys =
                BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK
                        | BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK
                        | BTM_LE_KEY_LENC;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
    {
		uint8_t i=0;
        /* save keys to NVRAM */
        p_keys = (uint8_t*) &p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram( LE_LOCAL_KEYS_VS_ID,
                sizeof(wiced_bt_local_identity_keys_t), p_keys, &result);
        MAIN_DEBUG("LE_BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT \r\n");
		for(i=0; i<16; i++)
		{
			MAIN_DEBUG("%02x ",p_keys[i]);
		}
		MAIN_DEBUG("\r\n");
    }
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
    {
        /* read keys from NVRAM */
	 uint8_t i=0;
        p_keys = (uint8_t *) &p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( LE_LOCAL_KEYS_VS_ID,
                sizeof(wiced_bt_local_identity_keys_t), p_keys, &result);
        MAIN_DEBUG("LE_BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT \r\n");
	for(i=0; i<16; i++)
	{
		MAIN_DEBUG("%02x ",p_keys[i]);
	}
		MAIN_DEBUG("\r\n");
    }
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                WICED_BT_SUCCESS);
        MAIN_DEBUG("LE_BTM_SECURITY_REQUEST_EVT \r\n");
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        MAIN_DEBUG("LE_Advertisement State Change: %d\r\n", *p_mode);
        if (*p_mode == BTM_BLE_ADVERT_OFF) {
            // hello_sensor_advertisement_stopped();
        }
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        p_ble_conn_param_update = &p_event_data->ble_connection_param_update;
        MAIN_DEBUG ("BTM BLE Connection Update event status:%d interval:%d latency:%d lsto:%d\n",
                            p_ble_conn_param_update->status,
                            p_ble_conn_param_update->conn_interval,
                            p_ble_conn_param_update->conn_latency,
                            p_ble_conn_param_update->supervision_timeout);

        break;

#endif
    default:
        MAIN_DEBUG("default\r\n");
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}




void button_init(void)
{

	#define cr_pad_fcn_ctl_adr4 0x003201b8
	#define REG32(x) (*(volatile unsigned*)(x))

	MAIN_DEBUG("REG32(cr_pad_fcn_ctl_adr4)  %x\r\n",REG32(cr_pad_fcn_ctl_adr4) );	
	unsigned val = REG32(cr_pad_fcn_ctl_adr4) & ~(0x0000F000);

	MAIN_DEBUG("val_0 %x\r\n",val);	
	REG32(cr_pad_fcn_ctl_adr4) = val | (7 << 12);
	MAIN_DEBUG("REG32(cr_pad_fcn_ctl_adr4)  %x\r\n",REG32(cr_pad_fcn_ctl_adr4) );	

	
	/* Configure the button available on the platform */
	wiced_hal_gpio_configure_pin( GPIO_MODE, GPIO_BUTTON_PULL_UP_SETTINGS( GPIO_EN_INT_FALLING_EDGE ), GPIO_BUTTON_DEFAULT_STATE );
	wiced_hal_gpio_configure_pin( GPIO_UP, GPIO_BUTTON_PULL_UP_SETTINGS( GPIO_EN_INT_FALLING_EDGE ), GPIO_BUTTON_DEFAULT_STATE );
	wiced_hal_gpio_configure_pin( GPIO_DOWN, GPIO_BUTTON_PULL_UP_SETTINGS( GPIO_EN_INT_FALLING_EDGE ), GPIO_BUTTON_DEFAULT_STATE );
	
	wiced_hal_gpio_configure_pin( GPIO_PTT, GPIO_BUTTON_PULL_UP_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), GPIO_BUTTON_DEFAULT_STATE );

	wiced_hal_gpio_configure_pin( GPIO_POWER_ON, GPIO_BUTTON_PULL_DOWN_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), GPIO_BUTTON_DEFAULT_STATE );
	wiced_hal_gpio_configure_pin( GPIO_LOW_BATTERY, GPIO_BUTTON_PULL_DOWN_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), GPIO_BUTTON_DEFAULT_STATE );

	wiced_hal_gpio_register_pin_for_interrupt(GPIO_MODE, app_interrupt_handler, NULL);
	wiced_hal_gpio_register_pin_for_interrupt(GPIO_UP, app_interrupt_handler, NULL);
	wiced_hal_gpio_register_pin_for_interrupt(GPIO_DOWN, app_interrupt_handler, NULL);
	wiced_hal_gpio_register_pin_for_interrupt(GPIO_POWER_ON, app_interrupt_handler, NULL);
	wiced_hal_gpio_register_pin_for_interrupt(GPIO_LOW_BATTERY, app_interrupt_handler, NULL);
	wiced_hal_gpio_register_pin_for_interrupt(GPIO_PTT, app_interrupt_handler, NULL);

	
	/*output*/
	wiced_hal_gpio_configure_pin( GPIO_POWER_EN, ( GPIO_OUTPUT_ENABLE | GPIO_PULL_DOWN),GPIO_PIN_OUTPUT_LOW);


	wiced_init_timer(&getSppAppData()->button_timer, &button_handler, 0, WICED_MILLI_SECONDS_TIMER);
	getSppAppData()->button_flag=0; 
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
    uint8_t iap2_service_uuid[] = {IAP2_ACCESSORY_UUID};
    uint16_t eir_length;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    //MAIN_DEBUG("hci_control_write_eir %x\n", pBuf);

    if (!pBuf)
    {
        MAIN_DEBUG("hci_iap2_spp_write_eir %x\n", pBuf);
        return;
    }

    p = pBuf;

   // length = strlen((char *)wiced_bt_cfg_settings.device_name);
   length=getSppAppData()->local_device_name_len;

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;        // EIR type full name
    //memcpy(p, wiced_bt_cfg_settings.device_name, length);
        memcpy(p, getSppAppData()->local_device_name, length);
    p += length;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_SERIAL_PORT & 0xff;
    *p++ = (UUID_SERVCLASS_SERIAL_PORT >> 8) & 0xff;

    *p++ = 16 + 1;                                  // length of 128 bit services + 1
    *p++ = BT_EIR_COMPLETE_128BITS_UUID_TYPE;       // 0x07 EIR type full list of 128 bit service UUIDs
    memcpy(p, iap2_service_uuid, 16);
    p += 16;

    *p++ = 0;                                       // end of EIR Data is 0

    eir_length = (uint16_t) (p - pBuf);
    
    // print EIR data
//    MAIN_DEBUG_ARRAY("EIR :", pBuf, p - pBuf);
    wiced_bt_dev_write_eir(pBuf, eir_length);
	wiced_bt_free_buffer(pBuf);

    return;
}



void spp_connection_fail_callback(void)
{
	MAIN_DEBUG("spp_connection_fail_callback, reconnect_cnt [%d]\r\n",reconnect_cnt);
	connect_fail_event(sdp_fail);

	if(reconnect_cnt<3)
	{
		reconnect_cnt++;
		wiced_start_timer(&getSppAppData()->reconnection_timer,1000);
		
	}
}

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    MAIN_DEBUG("%s handle:%d address:%B\r\n", __FUNCTION__, handle, bda);
    

	//pio_connection_led(true);
    getSppAppData()->spp_handle = handle;
    
	getSppAppData()->rx_data_flow=true;
	getSppAppData()->activeDevice =sppDevice;
	memcpy(getSppAppData()->remote_addr,bda,6);
    setSppState(sppDevConnected);

	last_device_store(sppdevice,bda);

    wiced_init_timer(&getSppAppData()->spp_transport_flow_control_timer, spp_transport_flow_control_timeout, handle, WICED_MILLI_SECONDS_TIMER);
    sendEventToUart(connection_success, 1, 0);
}

/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
    MAIN_DEBUG("%s handle:%d\r\n", __FUNCTION__, handle);
    getSppAppData()->spp_handle = 0;
 //   pio_connection_led(false);
    setSppState(sppDevConnectable);
	getSppAppData()->activeDevice =noActive;
    memset(getSppAppData()->remote_addr,0,6);
	sendEventToUart(disconnection_success, 1, 0);
	
}


void spp_connection_not_found_callback(void)
{
	 MAIN_DEBUG("spp_connection_not_found_callback\r\n");
	 connect_fail_event(not_find_service);
	 setSppState(sppDevDiscoverable);

	 if(reconnect_cnt<3)
	{
		last_device_connect();
		reconnect_cnt++;
	}
}


/*
* inquiry result callback
*/

 void spp_inquiry_callback(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data)
{
	MAIN_DEBUG("spp_inquiry_callback %d \r\n",p_inquiry_result->eir_complete_list);
	BD_ADDR addr;
	uint8 i;
	uint8_t *p;
	uint8 name_buf[50]; 
	uint8 len;



	if(p_inquiry_result->eir_complete_list==27) // inquring complete
	{
		sendEventToUart(inquiry_result_completed,1, 0);
		setSppState(sppDevConnectable);
		return;
	}
	
	p=p_eir_data;
	memset(name_buf,0,50);
	
	len=p[0]-1;
	if(p[0+1]=0x09)
	{
		if( len <=50)
		{
			MAIN_DEBUG("spp_inquiry_callback  addr : %B\r\n",&p_inquiry_result->remote_bd_addr);
			memcpy(name_buf,p+2,len);
			MAIN_DEBUG("spp_inquiry_callback len : [%d] name : ",len);
			//for(i=0; i< len; i++)
				MAIN_DEBUG("%s",name_buf);

			MAIN_DEBUG("\n");

			inquiry_result(p_inquiry_result->remote_bd_addr,name_buf,len);
			if(!wiced_is_timer_in_use(&getSppAppData()->inquiry_timer))
			{
				wiced_start_timer(&getSppAppData()->inquiry_timer, INQURIY_TIME);
			}
			getSppAppData()->inquiry_result=1;
		}
	}
	else
	{
		inquiry_result(p_inquiry_result->remote_bd_addr,0,0);
	}
	
	
		
	

	//wiced_bt_dev_vendor_specific_command(0x00B,8,feature,wiced_bt_dev_vendor_specific_command_callback);
}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    MAIN_DEBUG("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\r\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));

    MAIN_DEBUG("%s handle:%d len:%d %02x-%02x\r\n", __FUNCTION__, handle, data_len, p_data[0], p_data[data_len - 1]);



MAIN_DEBUG("queue size %d \r\n",UART_QUEUE_GET_DATA_LEN()+data_len);


uint8_t enqueue_data[1024]={0,};

#if BLE_ENABLE
uint16 len;
if(getSppAppData()->le_status==le_enable)
{
	len=data_len+4;
		
	enqueue_data[0] = 0x04;
	enqueue_data[1] = ((data_len+1)>>8)&0xff;
	enqueue_data[2] = (data_len+1)&0xff;
	enqueue_data[3] =  spp_iap_data_event;
	memcpy(&enqueue_data[4],p_data,data_len);
}
else
{
	len=data_len;
	memcpy(&enqueue_data,p_data,data_len);
}

	UART_ENQUEUE(enqueue_data,len);

#else
	UART_ENQUEUE(enqueue_data,data_len);

#endif


if((UART_QUEUE_GET_DATA_LEN())>UART_QUEUE_BUFFER_FLOW_OFF_SIZE)
{
	if(get_flow_status())
	{
		spp_flow_control(WICED_FALSE);

		/*Flow Control Timming issue protect*/
		wiced_start_timer(&getSppAppData()->spp_transport_flow_control_timer, 12);
        utilslib_delayUs(30*1000); 
	}
}

	
	if(getSppAppData()->uart_fun_flag==0)
	{
		high_speed_uart_send_proccess();
	}


	

    return WICED_TRUE;
}


void Rx_Flow_On_Retry(void)
{
	if(getSppAppData()->rx_flow_on_retry_count>2)
	{
		wiced_stop_timer(&getSppAppData()->rx_flow_on_retry_timer);
		getSppAppData()->rx_flow_on_retry_count=0;
		MAIN_DEBUG("Rx_Flow_On_Retry fail\r\n");
	}
	else
	{
		getSppAppData()->rx_flow_on_retry_count++;
		spp_flow_control(WICED_TRUE);
		MAIN_DEBUG("Rx_Flow_On_Retry count %d \r\n",getSppAppData()->rx_flow_on_retry_count);
	}
}



/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int bt_write_nvram(int nvram_id,void *p_data, int data_len)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    MAIN_DEBUG("NVRAM ID:%d written :%d bytes result:%d\r\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int bt_read_nvram(int nvram_id, void *p_data, int data_len)
{
	uint16_t        read_bytes = 0;
	wiced_result_t  result;


	read_bytes = wiced_hal_read_nvram(nvram_id, data_len, p_data, &result);
//	MAIN_DEBUG("NVRAM ID:%d read out of %d bytes:%d result:%d\r\n", nvram_id, data_len, read_bytes, result);

	return (read_bytes);
}


/*
 * Test function which start sending data.
 */
void app_interrupt_handler(void *data, uint8_t port_pin)
{
    //MAIN_DEBUG("gpio_interrupt_handler pin:%d level %d \r\n", port_pin,wiced_hal_gpio_get_pin_input_status(port_pin));


	switch(port_pin)
	{
		case GPIO_UP:
			getSppAppData()->button_flag = (getSppAppData()->button_flag | FLAG_GPIO_UP);
		
			break;
		case GPIO_DOWN:
			getSppAppData()->button_flag = (getSppAppData()->button_flag |FLAG_GPIO_DOWN);
		
			break;
		case GPIO_PTT:
			getSppAppData()->button_flag = (getSppAppData()->button_flag | FLAG_GPIO_PTT);
			
			getSppAppData()->ptt_flag=WICED_TRUE;
			break;
		case GPIO_MODE:
			getSppAppData()->button_flag = (getSppAppData()->button_flag | FLAG_GPIO_MODE);
			
			break;
		case GPIO_POWER_ON:
			getSppAppData()->button_flag = (getSppAppData()->button_flag |FLAG_GPIO_POWER_ON);
		
			break;
		case GPIO_LOW_BATTERY:
			
			getSppAppData()->low_battery_flag=WICED_TRUE;
			break;
		default:
		break;
	}

	if(!wiced_is_timer_in_use(&getSppAppData()->button_timer))
	{
		wiced_start_timer(&getSppAppData()->button_timer, BUTTON_TIMER_INTERVAL);
	}
     /* Get the status of interrupt on P# */
	wiced_hal_gpio_clear_pin_interrupt_status(port_pin);
}



#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
	MAIN_DEBUG("app_trace_callback %d  \r\n",type);
	uint8_t i;
	for(i=0;i<length;i++)
	MAIN_DEBUG("%02x",p_data[i]);

	MAIN_DEBUG("\r\n");
	

	return;	
}
#endif



void setSppState(const sppDevState state)
{
	MAIN_DEBUG("SPP State C=%d  N= %d \r\n",getSppAppData()->sppStatus, state);
	getSppAppData()->sppStatus = state;

	switch(state)
	{
		case sppDevInitialising:
		case sppDevReady:
		case sppDevConnectable:
		{
			m_led_flash(GPIO_LED_0, 100, 100, 3000, 1);
			
			wiced_bt_dev_set_discoverability(BTM_NON_DISCOVERABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);
			wiced_bt_dev_set_connectability(BTM_CONNECTABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);
			break;
		}
		case sppDevDiscoverable:
		{
			m_led_flash(GPIO_LED_0, 100, 100, 200, 1);
		
			wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);
			wiced_bt_dev_set_connectability(BTM_CONNECTABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);

			break;
		}
		case sppDevInquiring:
		{

			#if 0
			uint8_t                         mode;               /**< Inquiry mode (see #wiced_bt_inquiry_mode_e) */
			uint8_t                         duration;           /**< Inquiry duration (1.28 sec increments) */
			uint8_t                         filter_cond_type;   /**< Inquiry filter type  (see #wiced_bt_dev_filter_cond_e) */
			wiced_bt_dev_inq_filt_cond_t    filter_cond;        /**< Inquiry filter */
			#endif
				
			wiced_bt_dev_inq_parms_t inquiry_setteing;
			inquiry_setteing.mode=BTM_GENERAL_INQUIRY;
			inquiry_setteing.duration=0x0010; // 1.25s * n
			inquiry_setteing.filter_cond_type=BTM_CLR_INQUIRY_FILTER;
			//inquiry_setteing.filter_cond=null;
		
			wiced_bt_start_inquiry(&inquiry_setteing,&spp_inquiry_callback);
			break;
		}
		case sppDevConnecting:
		case sppDevConnected:
		case sppDevDisconnecting:
		{
			m_led_flash(GPIO_LED_0, 100, 100, 5000, 2);
			/*ConnectionWriteScanEnable(hci_scan_enable_off);*/
			wiced_bt_dev_set_discoverability(BTM_NON_DISCOVERABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);
			wiced_bt_dev_set_connectability(BTM_NON_CONNECTABLE, DISCOVERY_DURATATION, DISCOVERY_INTERVAL);
			break;
		}
		case sppDevSessionOpen:
		case sppDevSessionClose:
			break;
	}
	/*ConnectionWriteScanEnable(hci_scan_enable_page);*/
}

sppDevState GetSppState(void)
{
	return getSppAppData()->sppStatus;
}


bool lsbdaddr(wiced_bt_device_address_t addr_A, wiced_bt_device_address_t addr_B)
{
	uint8 i;
	for(i=0; i<6; i++)
	{
		if(addr_A[i]!=addr_B[i])
		{
			return false;
		}
	}
	return true;
}


void last_device_connect()
{
	 last_device_t        last_device;

	 MAIN_DEBUG("last_device_connect\r\n");
	if(bt_read_nvram(NVRAM_ID_LAST_DEVICE,&last_device,sizeof(last_device_t)))
	{
		BD_ADDR addr;
		uint8_t count;
		MAIN_DEBUG("last_device_connect: %B\r\n", last_device.addr);
		for(count=0; count<6; count++)
		{
			addr[count]=last_device.addr[5-count];
		}

		if(last_device.activeDevice==sppdevice)
		{
			wiced_bt_spp_connect(addr);
		}
		else
		{
			wiced_bt_iap2_connect(addr);
		}
	}

	
}

void pio_connection_led(bool mode)
{

#if 0
	if(getSppAppData()->pio_connection_detect_mode==PIO_CONNECTION_DETECT_ENABLE)
	{
		if(mode == true)
		{
			wiced_hal_gpio_set_pin_output(GPIO_CONNECTION_LED, GPIO_PIN_OUTPUT_HIGH);
			MAIN_DEBUG("pio_connection_led ON\r\n");
			
		}
		else
		{
			wiced_hal_gpio_set_pin_output(GPIO_CONNECTION_LED, GPIO_PIN_OUTPUT_LOW);
			MAIN_DEBUG("pio_connection_led OFF\r\n");
		}
	}
#endif
}


void inquiry_timer_fun(void)
{
	if(GetSppState()==sppDevInquiring)
	{
		if(getSppAppData()->inquiry_result=1)
		{
			wiced_start_timer(&getSppAppData()->inquiry_timer, INQURIY_TIME_ADD);
			getSppAppData()->inquiry_result=0;
		}
		else
		{
			setSppState(sppDevDiscoverable);
		}
	}
}


void add_bonded_device(wiced_bt_device_link_keys_t  device_link_key)
{
		wiced_bt_device_link_keys_t old_devie_link_key;
		wiced_bt_device_link_keys_t device_link;
		wiced_bool_t nvm_empty_flag = FALSE;
		uint8_t i;
		uint8_t bond_index;
		uint8_t last_bonded_index;



		 if(find_bonded_device( device_link_key.bd_addr,&device_link))
		 {
		 	remove_bonded_device(device_link_key.bd_addr);
		 }
		
		
		//add_devie_link_key=device_link_key;
		
		for(i=0; i<MAX_BONDED_DEVICE; i++)
		{
			if(!bt_read_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, &old_devie_link_key, sizeof(wiced_bt_device_link_keys_t)))
			{
				bt_write_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, &device_link_key,sizeof(wiced_bt_device_link_keys_t) );
				MAIN_DEBUG("add_bonded_device_1: [%B] \r\n",device_link_key.bd_addr);
				bt_read_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, &old_devie_link_key, sizeof(wiced_bt_device_link_keys_t));
				MAIN_DEBUG("add_bonded_device_2: [%B] \r\n",old_devie_link_key.bd_addr);
				nvm_empty_flag = TRUE;
				return;
			}
		}

		if(!nvm_empty_flag)
		{
			if(!bt_read_nvram(NVRAM_ID_BONDED_DEVICE_INDEX,&bond_index,sizeof(wiced_bt_device_link_keys_t)))
			{
				last_bonded_index=0;
				bt_write_nvram(NVRAM_ID_BONDED_DEVICE_BASE, &device_link_key, sizeof(wiced_bt_device_link_keys_t));
				bt_write_nvram(NVRAM_ID_BONDED_DEVICE_INDEX, &last_bonded_index, sizeof(last_bonded_index));
			}
			else
			{	
				last_bonded_index=bond_index+1;
				if(last_bonded_index==MAX_BONDED_DEVICE)
				{
					last_bonded_index=0;
				}
				bt_write_nvram(NVRAM_ID_BONDED_DEVICE_BASE+last_bonded_index, &device_link_key, sizeof(wiced_bt_device_link_keys_t));
				bt_write_nvram(NVRAM_ID_BONDED_DEVICE_INDEX, &last_bonded_index, sizeof(last_bonded_index));
			}
		}
}


wiced_bool_t find_bonded_device(wiced_bt_device_address_t addr,wiced_bt_device_link_keys_t *out_link_key)
{
	uint8_t i;

	for(i=0; i<MAX_BONDED_DEVICE; i++)
	{
		if(bt_read_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, out_link_key, sizeof(wiced_bt_device_link_keys_t)))
		{
			MAIN_DEBUG("find_bonded_device: NVRAM [%B]  address: [%B]\r\n",out_link_key->bd_addr, addr);
			if(lsbdaddr(&out_link_key->bd_addr, addr))
			{
				return WICED_TRUE;
			}
		}
	}
	return WICED_FALSE;
}

wiced_bool_t remove_bonded_device(wiced_bt_device_address_t addr)
{
	uint8_t i;
	wiced_bt_device_link_keys_t device_link;
	wiced_bool_t	result;

	for(i=0; i<MAX_BONDED_DEVICE; i++)
	{
		if(bt_read_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, &device_link, sizeof(wiced_bt_device_link_keys_t)))
		{
			MAIN_DEBUG("remove_bonded_device: NVRAM [%B]  address: [%B]\r\n",device_link.bd_addr, addr);
			if(lsbdaddr(device_link.bd_addr, addr))
			{
				wiced_hal_delete_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i, &result);
				return result;
			}
		}
	}
	return WICED_FALSE;
}

wiced_bool_t last_device_store(activeDeviceType type, BD_ADDR addr)
{
	last_device_t connection_device;
       connection_device.activeDevice=type;
       memcpy(connection_device.addr,addr,6);
      if(bt_write_nvram(NVRAM_ID_LAST_DEVICE,&connection_device,sizeof(last_device_t)))
      	{
	  	return WICED_TRUE;
      	}
	return WICED_FALSE;
}

void spp_flow_control(wiced_bool_t enable)
{
	wiced_bt_rfcomm_result_t i;
	if(getSppAppData()->spp_handle ==NULL)
	{
		return;
	}
	
	i= wiced_bt_spp_rx_flow_enable(getSppAppData()->spp_handle,enable);
	getSppAppData()->rx_data_flow=enable;
    
	MAIN_DEBUG("spp flow control %d \r\n",enable);
}

wiced_bool_t get_flow_status(void)
{
	return getSppAppData()->rx_data_flow;
}

//void send_uart_data(uint8* p_data,uint16 len)

void high_speed_uart_send_proccess(void)
{
	//wiced_hal_puart_synchronous_write(p_data,getSppAppData()->rx_data_len_sum);;

	uint8_t uart_tx_buffer[1024]={0,};
	uint16_t uart_size = getSppAppData()->uart_tx_size;
	uint16_t i=0;
	getSppAppData()->uart_fun_flag=1;
	//MAIN_DEBUG("uart_tx_timer_fun_start\r\n");
	i=UART_QUEUE_GET_DATA_LEN();
	MAIN_DEBUG("high_speed_uart_send_proccess %d\r\n",i );
	//DEBUG("high_speed_uart_send_proccess %d\r\n",i );
	//MAIN_DEBUG("uart_tx_timer_fun%d  ,   %d   \r\n",getUartQueue()->front ,getUartQueue()->rear);

#if 0// TTI not used uart
		
		if(wiced_hal_gpio_get_pin_input_status(GPIO_FLOW))
		{
			getSppAppData()->uart_fun_flag=0;
			return;
		}
		
		if(!send_uart_data())
		{
			MAIN_DEBUG("send_uart_data false \r\n" );
			getSppAppData()->uart_fun_flag=0;
			return;
		}
		
		if(i==0)
		{
			MAIN_DEBUG("high_speed_uart_send_proccess stop \r\n" );
			getSppAppData()->uart_fun_flag=0;
			return;
		}
	
		else if(i>uart_size)
       	{
		       MAIN_DEBUG("i>uart_size\r\n" );
			UART_DEQUEUE(getSppAppData()->rx_data_buffer,uart_size);
			
			getSppAppData()->rx_data_offset=getSppAppData()->rx_data_len=uart_size;
			send_uart_data();
       	}
		else
		{
		  	MAIN_DEBUG("uart_size>i\r\n" );
			UART_DEQUEUE(getSppAppData()->rx_data_buffer,i);  //getSppAppData()->rx_data_buffer will be  remove!
			
			getSppAppData()->rx_data_offset=getSppAppData()->rx_data_len=i;
			send_uart_data();
			
		}
		MAIN_DEBUG("high_speed_uart_send_proccess_2   %d\r\n",UART_QUEUE_GET_DATA_LEN());
		if(UART_QUEUE_GET_DATA_LEN()<UART_QUEUE_BUFFER_FLOW_ON_SIZE /*&&(!get_flow_status())*/)
		{
		    spp_flow_control(WICED_TRUE);
		    iap_flow_control(WICED_TRUE);
		}
		wiced_hal_wdog_restart();
		if(!wiced_hal_gpio_get_pin_input_status(GPIO_FLOW))
		{
			high_speed_uart_send_proccess();
		}
		else
		{
			getSppAppData()->uart_fun_flag=0;
		}
#endif
}


wiced_bool_t send_uart_data(void)
{
	MAIN_DEBUG("send_uart_data offset %d \r\n",getSppAppData()->rx_data_offset);
#if 0// TTI not used uart
	while((getSppAppData()->rx_data_offset>0) &&( !wiced_hal_gpio_get_pin_input_status(GPIO_FLOW)) /*&&getSppAppData()->rx_data_flow*/)
	{
		wiced_hal_puart_write(getSppAppData()->rx_data_buffer[getSppAppData()->rx_data_len-(getSppAppData()->rx_data_offset--)]);
		wiced_hal_wdog_restart();
	}

	MAIN_DEBUG("rx_data_offset %d\n",getSppAppData()->rx_data_offset);
	if(getSppAppData()->rx_data_offset==0)
	{
		 memset(getSppAppData()->rx_data_buffer,0,TRANS_UART_BUFFER_SIZE);
		 getSppAppData()->rx_data_len=0;
		 return WICED_TRUE;
	}
#endif
	return WICED_FALSE;
}


#if 1   // Uniquest
void spp_transport_flow_control_timeout(uint32_t param)
{
    MAIN_DEBUG("spp_transport_flow_control_timeout\n");
    //if( UART_QUEUE_GET_DATA_LEN() <= 5*1024)
    {
    	if(!getSppAppData()->rx_data_flow)
    	{
    		MAIN_DEBUG("rfcomm_buffer_read\n");
			rfcomm_buffer_read();
    	}
#if 0
    // time to tell library that we can accept data again
        spp_flow_control (WICED_TRUE);
        WICED_BT_TRACE("flow enable : 1\n");
#endif
	}
}
#endif


void link_key_updata_time_Fun(void)
{
	MAIN_DEBUG("link_key_updata_time_Fun \r\n");

	if(getSppAppData()->device_type_flag)
	{
		last_device_store(sppdevice,getSppAppData()->link_key_update_timer_data.link_key_updata.bd_addr);
	}
	if(getSppAppData()->link_key_update_timer_data.transport)
	{
		MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT _BR/EDR\r\n");
		add_bonded_device(getSppAppData()->link_key_update_timer_data.link_key_updata);
	}
	else
	{
		MAIN_DEBUG("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT _BLE\r\n");
		add_ble_bonded_device(getSppAppData()->link_key_update_timer_data.link_key_updata);
	}


	memset(&getSppAppData()->link_key_update_timer_data.link_key_updata,0,sizeof(getSppAppData()->link_key_update_timer_data.link_key_updata));
}

	
uint32 baudrate_converter(uint8 baud)
{
	uint32 result;
	switch(baud)
	{
		case UART_RATE_9600:
		{
			result = 9600;
			break;
		}
		case UART_RATE_38400:
		{
			result = 38400;
			break;
		}
		case UART_RATE_115200:
		{
			result = 115200;
			break;
		}
		case UART_RATE_230400:
		{
			result = 230400;
			break;
		}
		case UART_RATE_460800:
		{
			result = 460800;
			break;
		}
		case UART_RATE_921600:
		{
			result = 921600;
			break;
		}
		case UART_RATE_3000000:
		{
			result = 3000000;
			}

	}
	return result;	
}

void pairing_timer_fun(void)
{
 	setSppState(sppDevDiscoverable);
}

void reconnection_timer_fun(void)
{
 	last_device_connect();
}


void power_on(void)
{
	MAIN_DEBUG("Power ON()\r\n");
	wiced_hal_gpio_set_pin_output(GPIO_POWER_EN,GPIO_PIN_OUTPUT_HIGH);
	m_led_on(GPIO_LED_0);
	utilslib_delayUs(250*1000);
	m_led_off(GPIO_LED_0);
	m_led_flash(GPIO_LED_1, 100, 100, 3000, 1);	
	setSppState(sppDevConnectable);
}


