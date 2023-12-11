/*
 * Madgwick_filter.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Hamza
 */

#ifndef INC_MADGWICK_FILTER_H_
#define INC_MADGWICK_FILTER_H_

//----------------------------------------------------------------------------------------------------
// Variable declaration
#define PI 3.141592
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* INC_MADGWICK_FILTER_H_ */
