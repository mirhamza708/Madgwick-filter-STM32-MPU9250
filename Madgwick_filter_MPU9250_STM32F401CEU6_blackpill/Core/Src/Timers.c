/*
 * Timers.c
 *
 *  Created on: Nov 30, 2023
 *      Author: Hamza
 */

#include "Timers.h"


volatile bool read_mpu_data_flag = false;
volatile bool uart_send_data_flag = false;
volatile bool read_ak_data_flag = false;
volatile bool execute_filter_flag = false;

volatile uint8_t mpu_read_counter = 0;
volatile uint8_t ak_read_counter = 0;
volatile uint8_t uart_send_data_counter = 0;
volatile uint8_t filter_counter = 0;

/*
 * Function: HAL_TIM_PeriodElapsedCallback
 * ----------------------------
 *   Handles the interrupts which are fired at 1KHz when timer is up.
 *   Sensor data read timing is made out of this single timer.
 *
 *   htim: pointer to the data structure of the timer handle.
 *
 *   returns: void
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim2)
	{
		ak_read_counter++;
		uart_send_data_counter++;
		filter_counter++;

//		execute_filter_flag = true;
		if(filter_counter == 2)
		{
			execute_filter_flag = true;
			filter_counter = 0;
		}

		if(ak_read_counter == 10)
		{
			read_ak_data_flag = true;
			ak_read_counter = 0;
		}

		if(uart_send_data_counter == 20)
		{
			uart_send_data_flag = true;
			uart_send_data_counter = 0;
		}
	}
}
