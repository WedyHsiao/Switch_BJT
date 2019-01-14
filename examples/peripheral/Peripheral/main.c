/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 * 
 */
 
 /**wedy claim
 *@brief this development is for the multiple functions with below:
 *Function_BUTTON
 *Function_ADC
 *Function_VIBRATOR
 *Function_PWM
 *Function_EVENT
 *please use above symbol to define the function
 *each time re-define must rebuild the program then load
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h" // serial port display
#include "app_error.h" //for UART notify
#include "nrf_delay.h" //time delay
#include "nrf_gpio.h" //gpio control and setting
#include "nrf.h" //identify the software
#include "bsp.h" //button for functionally module
#include "app_timer.h" //add for the event control
#include "nrf_drv_gpiote.h" //gpiote control
#include "nrf_drv_clock.h" //add for the FCLK for event
#include "app_button.h" //event control 

////////////////////////////////////////////Define///////////////////////////

/* Button definitions */
const uint8_t buttons_list[BUTTONS_NUMBER]={BUTTON_1,BUTTON_2,BUTTON_3,BUTTON_4};   //there are three buttons in use
const uint8_t leds_list[LEDS_NUMBER]={LED_1,LED_2,LED_3,LED_4}; // the LED in use

//Wedy add for USB adapter
const uint8_t pen_list[PEN_NUMBER]= {PEN_1,PEN_2,PEN_3};

/*UART definitions*/
//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

/*Eevent definition*/
#define APP_TIMER_PRESCALER 0 /**< Value of the RTC1 PRESCALER register. */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */

////////////////////////////////////End fo Define///////////////////////////

////////////////////////////////////////////UART//////////////////
/*Wedy add UART control*/
/*baudrate=115200, 8 component(view in the content)*/
void uart_error_handle(app_uart_evt_t * p_event)
{
 if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
 {
 APP_ERROR_HANDLER(p_event->data.error_communication);
 }
 else if (p_event->evt_type == APP_UART_FIFO_ERROR)
 {
  APP_ERROR_HANDLER(p_event->data.error_code);
 }
}
////////////////////////////////////////////UART//////////////////

//////////////////VIBRATOR setting/////////////////////////////////////
/*vibrating counts*/
#if Function_VIBRATOR
static void VIBRATOR_Init(void)    //set the pin control
{
 //simulate for vibrator power consumption
 nrf_gpio_cfg_output(Vibrator);   //set vibrator	
 nrf_gpio_pin_clear(Vibrator);
	
}
//////////////////VIBRATOR setting/////////////////////////////////////

//////////////////////////////////USB adapter////////////////////////////
#elif Function_PEN
static void PEN_Init (void)
{
	 nrf_gpio_cfg_output(LED_3);                            //LED3 for PEN testing
	 nrf_gpio_pin_set(LED_3);
	 nrf_gpio_range_cfg_output(PEN_START,PEN_STOP); //P16,P17,P18
	 for(int i=0;i<PEN_NUMBER;i++)											//set the pen pin 
		{
		nrf_gpio_pins_clear (pen_list[i]);
		}
}
//////////////////////////////////USB adapter////////////////////////////
#else
#error " please idenfify a function!"
#endif


/*UART content */
static void UART_Init(void)
{	
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200 //the baudrate is set
      };

		
    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);			
}
/*UART content End*/

/*******
*******
******
*****
***
**
 * @brief Function for main application entry.
 */
int main(void)                 
{
	UART_Init();
	uint32_t err_code;
	
#if Function_VIBRATOR  //Wedy vibrator power consumption test
		VIBRATOR_Init();
		for(int i=1;i<=5000;i++)
		{
			int tick=0;
			tick+=i;
			printf("%d\r\r", i);
			nrf_gpio_pin_toggle(Vibrator);
			nrf_delay_ms(500);
		}
		
#elif Function_PEN
		PEN_Init();
		for(int i=1;i<=5000;i++)
		{
			int tick=0;
			tick+=i;
			printf("%d\r\n", i);
			nrf_gpio_pin_write(PEN_1, 1);
			nrf_gpio_pin_write(PEN_2, 1);
			nrf_gpio_pin_write(PEN_3, 1);
			nrf_delay_ms(5000);
			nrf_gpio_pin_write(PEN_1, 0);
			nrf_gpio_pin_write(PEN_2, 0);
			nrf_gpio_pin_write(PEN_3, 0);
			nrf_delay_ms(5000);
		}
		nrf_gpio_pin_toggle(LED_3);
		printf("End of usb adapter charger test!");
		
#else
#error	"Function is not defined, please define the preprocessor symbol in the file! "	
#endif
	}


/** @} */
