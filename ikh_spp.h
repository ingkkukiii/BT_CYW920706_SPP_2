/*
 * ikh_spp.h
 *
 *  Created on: 2022. 11. 29.
 *      Author: user
 */

#ifndef IKH_SPP_H_
#define IKH_SPP_H_


/** \addtogroup Platfrom config - Peripherals pin configuration
*   \ingroup HardwareDrivers
*/
/*! @{ */

/******************************************************
 *                   Enumerations
 ******************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_gpio.h"

extern void platform_led_init( void );
extern void platform_puart_flow_control_init( void );

typedef struct
{
    wiced_bt_gpio_numbers_t led_gpio;
    uint32_t led_config;
    uint32_t led_default_state;
}wiced_led_config_t;

typedef enum
{
    WICED_PLATFORM_LED_1,
    WICED_PLATFORM_LED_MAX
}wiced_platform_led_t;

#define HCI_UART_DEFAULT_BAUD   115200   /* default baud rate is 3M, that is max supported baud rate on Mac OS */

#ifdef LEGACY_BOARD //CYW920706WCDEVAL
#define WICED_GPIO_LED                                             WICED_P31

#define WICED_GPIO_BUTTON_TOGGLE                                   WICED_P30    
#define WICED_GPIO_PIN_BUTTON_TOGGLE                               WICED_GPIO_BUTTON
#define WICED_GPIO_BUTTON_ONOFF                                    WICED_P34    
#define WICED_GPIO_PIN_BUTTON_2                                    WICED_GPIO_BUTTON_ONOFF

/* x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | x )
/* if edge isn't specified (legacy/shared code), use GPIO_EN_INT_FALLING_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS_DEFAULT                  ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE )
#define WICED_GPIO_BUTTON_DEFAULT_STATE                     GPIO_PIN_OUTPUT_LOW

//P49_20706 (default)
#define WICED_BUTTON_PRESSED_VALUE                 1
#else //MOVON_BOARD
#define WICED_GPIO_LED                                             WICED_P12
#define WICED_GPIO_BUTTON_TOGGLE                                   WICED_P38            
#define WICED_GPIO_PIN_BUTTON_TOGGLE                               WICED_GPIO_BUTTON
#define WICED_GPIO_BUTTON_ONOFF                                    WICED_P34            
#define WICED_GPIO_PIN_BUTTON_2                                    WICED_GPIO_BUTTON_ONOFF

/* x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | x )
/* if edge isn't specified (legacy/shared code), use GPIO_EN_INT_RISING_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS_DEFAULT                  ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_RISING_EDGE )
#define WICED_GPIO_BUTTON_DEFAULT_STATE                     GPIO_PIN_OUTPUT_LOW


//P49_20706 (default)
#define WICED_BUTTON_PRESSED_VALUE                 0
#endif

#define WICED_PUART_TXD                           WICED_P31      /* pin for PUART TXD         */
#define WICED_PUART_RXD                           WICED_P02      /* pin for PUART RXD         */

#if PUART_RTS_CTS_FLOW
#define WICED_PUART_CTS                           WICED_P35      /* pin for PUART CTS         */
#define WICED_PUART_RTS                           WICED_P30      /* pin for PUART RTS         */
#else
#define WICED_PUART_CTS  0
#define WICED_PUART_RTS  0
#endif
/* @} */

/** \addtogroup Platfrom config - Default flash(i.e. flash exists on WICED eval boards) configuration.
*   \ingroup HardwareDrivers
*/
/*! @{ */
/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 *  Recommend to use 4K sector flash.
 */
#if defined(USE_256K_SECTOR_SIZE)
  #define FLASH_SECTOR_SIZE         (256*1024)
  #define FLASH_SIZE                (256*FLASH_SECTOR_SIZE)
#else
  #define FLASH_SECTOR_SIZE         (4*1024)
  #define FLASH_SIZE                0x80000 // 512  kbyte/4M Bit Sflash for new tag boards
#endif

/** Number of sectors reserved from the end of the flash for the application
 *  specific purpose(for ex: to log the crash dump). By default no reservation
 Note:- 16K of flash is used for internal firmware operation.
 (Remaining of the flash - reservation) can be divided equally and used for active
 and upgradable firmware. So, application should care the OTA firmware(application+patch)
 size while reserving using below.
 */
#define APPLICATION_SPECIFIC_FLASH_RESERVATION  0

/**
 * platform_puart_flow_control_pin_init
 *
 * Unbond pads to prepare RTS/CTS flow control
 */
void platform_puart_flow_control_pin_init(void);

/**
 * Platform-specific functions to disable pads bonded to pins
 *
*/
void unbond_P11(void);
void unbond_P30(void);
void unbond_P35(void);

/* @} */



#endif /* IKH_SPP_H_ */
