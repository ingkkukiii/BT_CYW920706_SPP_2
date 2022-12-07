

#include "sparcommon.h"
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_timer.h"
#include "spp_debug.h"
#include "spp_main.h"
#include "spp_private.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_gpio.h"
#include "spp_gpio.h"
#include "m_led.h"


#ifdef DEBUG_BUTTON
#define BUTTON_DEBUG           DEBUG
#define BUTTON_DEBUG_ARRAY     DEBUG_ARRAY
#define BUTTON_DEBUG_CRIT      DEBUG_CRIT
#else
#define BUTTON_DEBUG         
#define BUTTON_DEBUG_ARRAY     
#define BUTTON_DEBUG_CRIT     
#endif

void button_handler(void)
{
	//MAIN_DEBUG("button_handler button flag %d\r\n",getSppAppData()->button_flag);
	static uint16_t gpio_up_count=0;
	static uint16_t gpio_down_count=0;
	static uint16_t gpio_mode_count=0;
	static uint16_t power_on_count=0;
	static uint8 mode_flag = BUTTON_MODE_NONE;
	

	if(getSppAppData()->low_battery_flag==WICED_TRUE)
	{
		getSppAppData()->low_battery_flag=WICED_FALSE;
		if(!wiced_hal_gpio_get_pin_input_status(GPIO_LOW_BATTERY))
		{
				m_led_flash(GPIO_LED_1, 200, 100, 1000, 1);
		}
		else
		{
				m_led_flash(GPIO_LED_1, 100, 100, 3000, 1);
		}
	}


		if(wiced_hal_gpio_get_pin_input_status(GPIO_POWER_ON))
		{
				power_on_count++;
				//BUTTON_DEBUG("GPIO_POWER_ON %d/%d\r\n",power_on_count,BUTTON_LONG_PRESS_COUNT);

				if(power_on_count == BUTTON_VERY_LONG_PRESS_COUNT)
				{
					
					BUTTON_DEBUG("GPIO_POWER_ON  BUTTON_VERY_LONG_PRESS_COUNT \r\n");
					getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_POWER_ON);
					power_on_count=0;
					wiced_result_t result;
					uint8_t i;

					wiced_hal_delete_nvram(NVRAM_ID_LAST_DEVICE+i,&result);
		

					for(i=0; i<MAX_BONDED_DEVICE+1;i++)
					{
						wiced_hal_delete_nvram(NVRAM_ID_BONDED_DEVICE_BASE+i-1,&result);
					}
					
					setSppState(sppDevDiscoverable);
				}
				else if(power_on_count == BUTTON_LONG_PRESS_COUNT)
				{
					BUTTON_DEBUG("GPIO_POWER_ON  BUTTON_LONG_PRESS_COUNT \r\n");
					if(getSppAppData()->power_on_flag)
					{
						power_on();
						getSppAppData()->power_on_flag = false;
					}
					else
					{
						BUTTON_DEBUG("Power OFF \r\n");
						getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_POWER_ON);
						power_on_count=0;
						wiced_bt_spp_disconnect(getSppAppData()->spp_handle);
						led_off_all();
					
						m_led_on(GPIO_LED_0);
						utilslib_delayUs(250*1000);
						m_led_off(GPIO_LED_0);
						utilslib_delayUs(250*1000);
						m_led_on(GPIO_LED_0);
						utilslib_delayUs(250*1000);
						m_led_off(GPIO_LED_0);
												
						wiced_hal_gpio_set_pin_output(GPIO_POWER_EN,GPIO_PIN_OUTPUT_LOW);
					}
				}
					
		}
		else
		{
				BUTTON_DEBUG("wiced_hal_gpio_get_pin_input_status -> 0 \r\n");
				getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_POWER_ON);
				if(power_on_count < BUTTON_VERY_LONG_PRESS_COUNT && power_on_count >= BUTTON_LONG_PRESS_COUNT)
				{
					last_device_connect();
				}
				power_on_count=0;
		}

	

	if(getSppAppData()->ptt_flag==WICED_TRUE)
	{	
		static uint8_t debounce_count=0;


		//if(debounce_count == 0)
		{
			if(!wiced_hal_gpio_get_pin_input_status(GPIO_PTT))
			{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_ptt_press, 1);

			}
			else
			{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_ptt_release, 1);

			}
			//debounce_count++;
		}
		//else
		{
		//	if(debounce_count == 1)
			{
			//	debounce_count=0;
				getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_PTT);
				getSppAppData()->ptt_flag=WICED_FALSE;
			}
			//else
			{
		//		debounce_count++;
			}
			
		}
		
		
		
		
	}
	

	
	if(wiced_hal_gpio_get_pin_input_status(GPIO_UP))
	{
		getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_UP);

		if(gpio_up_count>0)
		{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_up_short_press, 1);
		}

		gpio_up_count=0;
	}
	else
	{
		gpio_up_count++;
		if(gpio_up_count>BUTTON_LONG_PRESS_COUNT)
		{
			gpio_up_count=0;
			getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_UP);
			
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_up_long_press, 1);
		}
	}

	
	if(wiced_hal_gpio_get_pin_input_status(GPIO_DOWN))
	{
		getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_DOWN);

		if(gpio_down_count>0)
		{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_down_short_press, 1);
		}

		gpio_down_count=0;
	}
	else
	{
		gpio_down_count++;
		if(gpio_down_count>BUTTON_LONG_PRESS_COUNT)
		{
			gpio_down_count=0;
			getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_DOWN);
			
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_down_long_press, 1);
		}
	}


	if(getSppAppData()->ptt_flag==WICED_TRUE)
	{
		getSppAppData()->ptt_flag=WICED_FALSE;
		if(!wiced_hal_gpio_get_pin_input_status(GPIO_PTT))
		{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_ptt_press, 1);
		}
		else
		{
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_ptt_release, 1);
		}
	}

	//BUTTON_DEBUG("wiced_hal_gpio_get_pin_input_status %d\r\n",wiced_hal_gpio_get_pin_input_status(GPIO_MODE));
	if(!wiced_hal_gpio_get_pin_input_status(GPIO_MODE))
	{	
		if(mode_flag == BUTTON_MODE_NONE)
		{
			mode_flag = BUTTON_MODE_ONE_CLICK;
		}
		else if(mode_flag == BUTTON_MODE_ONE_CLICK_RELEASE)
		{
			mode_flag = BUTTON_MODE_DOUBLE_CLICK;
		}
	}
	else
	{
		if(mode_flag == BUTTON_MODE_ONE_CLICK)
		{
			mode_flag = BUTTON_MODE_ONE_CLICK_RELEASE;
		}
		else if(mode_flag == BUTTON_MODE_ONE_CLICK_RELEASE)
		{
			gpio_mode_count++;

			if(gpio_mode_count>BUTTON_DOUBLE_CLICK_COUNT)
			{
				getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_MODE);
				wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_mode_press, 1);
				mode_flag=BUTTON_MODE_NONE;
				gpio_mode_count = 0;
			}
			
		}
		else if(mode_flag == BUTTON_MODE_DOUBLE_CLICK)
		{
			getSppAppData()->button_flag = getSppAppData()->button_flag & (~FLAG_GPIO_MODE);
			wiced_bt_spp_send_session_data(getSppAppData()->spp_handle, &b_mode_double_press, 1);
			mode_flag=BUTTON_MODE_NONE;
			gpio_mode_count = 0;
		}
	}

	//BUTTON_DEBUG("button_handler button flag %d\r\n",getSppAppData()->button_flag);
	
	if(getSppAppData()->button_flag)
	{
		wiced_start_timer(&getSppAppData()->button_timer, BUTTON_TIMER_INTERVAL);
	}

}

