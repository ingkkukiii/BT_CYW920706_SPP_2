

#ifndef SPP_MAIN_H
#define SPP_MAIN_H

#include "spp_private.h"
#include "wiced_timer.h"
#include "wiced_transport.h"

#define SPP_RFCOMM_SCN                  1 // 2 Rfcomm channel modify for winsocket 
extern wiced_transport_buffer_pool_t* host_trans_pool;

#define LINK_KEY_UPDATE_TIMEOUT		100

//#define HCI_TRACE_OVER_TRANSPORT            0    // If defined HCI traces are send over transport/WICED HCI interface
//#define LOOPBACK_DATA                     1   // If defined application loops back received data

#define WICED_EIR_BUF_MAX_SIZE              264

#define DEVICE_NAME_MAX_LEN		20
#define IAP_MANUFATURE_MAX_LEN		50
#define IAP_SERIAL_NUMBER_MAX_LEN	20
#define IAP_MODEL_NUMBER_MAX_LEN	20



/*inquiry time*/
#define INQURIY_TIME		15000
#define INQURIY_TIME_ADD	3000


/*device setting nvm */
#define NVRAM_ID_START                  			0x50
#define NVRAM_ID_LAST_DEVICE                       	0x50
#define NVRAM_ID_DEVICE_NAME 				0x51
#define NVRAM_ID_DEVICE_NAME_LEN 			0x52
#define NVRAM_ID_UART_BAUDLATE			0x53
#define NVRAM_ID_BUTTON_MODE				0x54
#define NVRAM_ID_PINCODE_VALUE			0x55
#define NVRAM_ID_PINCODE_ENABLE			0x56
#define NVRAM_ID_AUTO_CONNECTION			0x57
#define NVRAM_ID_PIO_CONNECTION_DETECT	0x58
#define NVRAM_ID_NUMERIC_ENABLE			0x59
#define NVRAM_ID_LE_STATUS				0x60


#define NVRAM_ID_END					NVRAM_ID_LE_STATUS
#define MAX_BONDED_DEVICE					(10)
#define NVRAM_ID_BONDED_DEVICE_INDEX		0x30
#define NVRAM_ID_BONDED_DEVICE_BASE		0x31


//#ifdef BLE_ENABLE
		
#define NVRAM_ID_BLE_BONDED_DEVICE_INDEX		0x20
#define NVRAM_ID_BLE_BONDED_DEVICE_BASE			0x21
#define NVRAM_ID_BLE_DEVICE_NAME 				0x2d
#define NVRAM_ID_BLE_DEVICE_NAME_LEN 			0x2e
#define NVRAM_ID_BLE_READ_DATA					0x2f
#define NVRAM_ID_BLE_READ_DATA_LEN				0x30

#define NVRAM_ID_BLE_START  					NVRAM_ID_BLE_DEVICE_NAME
#define NVRAM_ID_BLE_END						NVRAM_ID_BLE_DEVICE_NAME_LEN
//#endif



/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER                       		1017
#define TRANS_MAX_BUFFERS                   	10
#define TRANS_UART_BUFFER_SIZE              1024*1
#define SPP_MAX_PAYLOAD                     	1007 //1007










typedef enum
{
	sdp_fail=0,
	not_find_service
}connection_fail_reaon;

typedef enum
{
	SPP_DEACTIVATED = 0,
	SPP_ACTIVATED
}sppStream; 

typedef enum
{
      BUTTON_ENABLE = 0,
    BUTTON_DISABLE = 1
}sppButtonMode;

typedef enum
{
	PIO_EVENT_ENABLE = 0,
	PIO_EVENT_DISABLE = 1
}pioEventMode;

typedef enum
{
	PIO_CONNECTION_DETECT_ENABLE = 0,
	PIO_CONNECTION_DETECT_DISABLE = 1
}pioConnectionDetect;


typedef enum
{
  pincode_enable,
  pincode_disable	
} PinCodeState;


typedef enum
{
  le_enable,
  le_disable	
} Lestatus_t;

typedef enum
{
  numeric_enable,
  numeric_disable	
} NumericState;


typedef enum
{
	noActive,
	sppDevice,
	iPhone,
	iPad
}activeDeviceType;



typedef enum
{
    sppDevInitialising,
    sppDevReady,
    sppDevDiscoverable,
    sppDevInquiring,
    sppDevConnecting,
    sppDevConnected,
    sppDevDisconnecting,
    sppDevConnectable,
     sppDevSessionOpen,
    sppDevSessionClose,
} sppDevState;


/*
typedef struct
{
       uint8_t isbond[MAX_BONDED_DEVICE];
}bonded_device_index;
*/

typedef struct
{
       activeDeviceType	activeDevice;
	BD_ADDR  addr;	
}last_device_t;


typedef struct
{
	wiced_bt_device_link_keys_t  link_key_updata;
	wiced_bt_transport_t		 transport;
}link_key_update_timer_data_t;

typedef struct
{
    sppDevState           	sppStatus;

    activeDeviceType	activeDevice;
   BD_ADDR			remote_addr;
   BD_ADDR			LE_connecting_addr;
    wiced_timer_t           inquiry_timer;
    uint8				inquiry_result; 
    uint16_t                	spp_handle;
    uint16_t    			 iap_ea_handle;            // session handle of the current EA connection


   uint8_t				local_device_name[DEVICE_NAME_MAX_LEN];
   uint8_t				local_device_name_len;

	 wiced_timer_t   	spp_transport_flow_control_timer;
	 wiced_timer_t 		link_key_update_timer;
	 link_key_update_timer_data_t	link_key_update_timer_data;	
	 

#if 1



    uint16_t conn_id;                  // connection ID referenced by the stack
    uint16_t peer_mtu;                 // peer MTU
    uint8_t num_to_write;          // num msgs to send, incr on each button intr
    uint8_t flag_indication_sent;     // indicates waiting for ack/cfm
    uint8_t flag_stay_connected; // stay connected or disconnect after all messages are sent

#endif
   
	uint8_t							iap_manufacture[IAP_MANUFATURE_MAX_LEN];
	uint8_t							iap_manufacture_len;
	uint8_t							iap_serialnumber[IAP_SERIAL_NUMBER_MAX_LEN];
	uint8_t							iap_serialnumber_len;
	uint8_t							iap_modelnumber[IAP_MODEL_NUMBER_MAX_LEN];
	uint8_t							iap_modelnumber_len;
	
	
	uint8_t							iap_sw_version[4];
    uint8_t							iap_hw_version[4];
	uint8_t							iap_1_sw_version[4];
    uint8_t							iap_1_hw_version[4];


	uint8_t				local_Le_device_name[DEVICE_NAME_MAX_LEN];
   uint8_t				local_Le_device_name_len;
   uint8_t				ble_read_data[20];
   uint16_t				ble_read_data_len;

   uint16_t			rx_data_offset;
   uint8_t				rx_data_buffer[TRANS_UART_BUFFER_SIZE];
      uint16_t			rx_data_len;
   wiced_bool_t		rx_data_flow;
    wiced_timer_t           uart_flow_timer;

   sppButtonMode		button_mode;
   uint8_t 				pincode[4];
   uint8			pincode_enable;
   uint8			pincode_flag;
   uint8			numeric_status;

   wiced_timer_t		rx_flow_on_retry_timer;
  uint8_t				rx_flow_on_retry_count;
   
  wiced_timer_t		 uart_tx_timer;
  uint8 				pio_connection_detect_mode;

  uint8			le_status;
  uint8_t			authentication_reject_evet_flag;
  uint8_t			uart_fun_flag;

  uint8_t			uart_timeout;
  uint16_t			uart_tx_size;


  wiced_bool_t		device_type_flag;


  wiced_timer_t		pairing_timer;	
  wiced_timer_t		button_timer;

  wiced_timer_t		reconnection_timer;
  
  uint16_t			button_flag;
  wiced_bool_t		ptt_flag;
  wiced_bool_t		low_battery_flag;
  wiced_bool_t		power_on_flag;
}appSppData;



/*******************************************************************
 * Function Prototypes
 ******************************************************************/

void Rx_Flow_On_Retry(void);

void button_init(void);


appSppData* getSppAppData(void);
void app_write_eir(void);
void spp_rx_timer(void);
bool lsbdaddr(wiced_bt_device_address_t addr_A, wiced_bt_device_address_t addr_B);
void last_device_connect(void);
sppDevState GetSppState(void);
void inquiry_timer_fun(void);
void pairing_timer_fun(void);
void reconnection_timer_fun(void);

void flow_timer_fun(void);
void uart_send_extra_data(void);

void spp_flow_control(wiced_bool_t enable);
wiced_bool_t get_flow_status(void);

wiced_bool_t find_bonded_device(wiced_bt_device_address_t addr,wiced_bt_device_link_keys_t *out_link_key);
void add_bonded_device(wiced_bt_device_link_keys_t  device_link_key);
wiced_bool_t remove_bonded_device(wiced_bt_device_address_t addr);
wiced_bool_t last_device_store(activeDeviceType type, BD_ADDR addr);

//void send_uart_data(uint8* p_data,uint16 len);
wiced_bool_t send_uart_data(void);
void high_speed_uart_send_proccess(void);
uint32 baudrate_converter(uint8 baud);

void uart_timer_config(void);

 void wiced_bt_dev_vendor_specific_command_callback (wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params);

#endif
