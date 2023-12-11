/*
 * Timers.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Hamza
 */

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

#include "main.h"
#include "MPU9250.h"

extern TIM_HandleTypeDef htim2;

extern volatile bool read_mpu_data_flag;
extern volatile bool uart_send_data_flag;
extern volatile bool read_ak_data_flag;
extern volatile bool execute_filter_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);



#endif /* INC_TIMERS_H_ */
