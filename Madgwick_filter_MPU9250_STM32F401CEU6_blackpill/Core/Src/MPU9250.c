/*
 * MPU9250.c
 *
 *  Created on: Nov 29, 2023
 *      Author: Hamza
 */
#include "main.h"
#include "MPU9250.h"
#include "Timers.h"

//startup variables
static volatile bool i2c1_tx_cplt = false;
static volatile bool i2c1_rx_cplt = false;
uint8_t i2c_rx_data = 0;
uint8_t i2c_tx_data = 0;

//runtime vaiables
static volatile bool i2c1_xfer_in_progress = false;
static volatile bool mpu6500_drdy_flag = false;
static volatile bool ak8963_drdy_flag = false;
static volatile bool ak8963_strdy_flag = false;
static volatile bool mpu6500_data_read_started = false;
static volatile bool ak8963_status_read_started = false;
static volatile bool ak8963_data_read_started = false;

const int16_t tX[3] = {0,  1,  0};
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};

struct _MPU9250 MPU9250;

/*
 * Function: max
 * ----------------------------
 *   Find maximum between two numbers.
 *
 *   returns: maximum between the two input arguments
 */
int16_t max(int16_t num1, int16_t num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

/*
 * Function: min
 * ----------------------------
 *   Find minimum between two numbers.
 *
 *   returns: minimum between the two input arguments
 */
int16_t min(int16_t num1, int num2)
{
    return (num1 > num2 ) ? num2 : num1;
}
/*
 * Function: mpu_i2c_writeregister
 * ----------------------------
 *   Write 1 byte data to a register address of mpu6500
 *
 *   address: address of register to which data is written.
 *   data:	  points to a byte location from which stores data to be sent.
 *
 *   returns: void
 */
void mpu_i2c_writeregister(uint8_t address, uint8_t * data)
{
	i2c1_tx_cplt = false;
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Write_DMA(&hi2c1, MPU9250_ADDRESS, address, MEM_ADD_SIZE, data, REG_MEM_SIZE);
	while(i2c1_tx_cplt != true){}
}

/*
 * Function: mpu_i2c_writeregister
 * ----------------------------
 *   Read 1 byte data from a register address of mpu6500
 *
 *   address: address of register from which data is read.
 *   data:	  points to a byte location where read data is stored.
 *   		  this location should be a globally defined variable.
 *
 *   returns: void
 */
void mpu_i2c_readregister(uint8_t address, uint8_t * data)
{
	i2c1_rx_cplt = false;
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU9250_ADDRESS, address, MEM_ADD_SIZE, data, REG_MEM_SIZE);
	while(i2c1_rx_cplt != true){}

}

/*
 * Function: mpu_i2c_writeregister
 * ----------------------------
 *   Write 1 byte data to a register address of ak8963
 *
 *   address: address of register to which data is written.
 *   data:	  points to a byte location from which stores data to be sent.
 *
 *   returns: void
 */
void ak_i2c_writeregister(uint8_t address, uint8_t * data)
{
	i2c1_tx_cplt = false;
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Write_DMA(&hi2c1, AK8963_ADDRESS, address, MEM_ADD_SIZE, data, REG_MEM_SIZE);
	while(i2c1_tx_cplt != true){}

}

/*
 * Function: mpu_i2c_writeregister
 * ----------------------------
 *   Read 1 byte data from a register address of ak8963
 *
 *   address: address of register from which data is read.
 *   data:	  points to a byte location where read data is stored.
 *   		  this location should be a globally defined variable.
 *
 *   returns: void
 */
void ak_i2c_readregister(uint8_t address, uint8_t * data)
{
	i2c1_rx_cplt = false;
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Read_DMA(&hi2c1, AK8963_ADDRESS, address, MEM_ADD_SIZE, data, REG_MEM_SIZE);
	while(i2c1_rx_cplt != true){}
}

/*
 * Function: MPU9250_init
 * ----------------------------
 *   Initializes both the sensors included in the MPU9250. Also starts the
 *   the timer for timing the read operations from both sensors.
 *
 *   returns: Currently returns two error codes if the sensors are not identified
 */
mpuinit_error_codes MPU9250_init()
{
	MPU9250.first_start = 1;
	MPU9250.gyro_calibration_done = 0;

	MPU9250.gx_offset = 3;
	MPU9250.gy_offset = -5;
	MPU9250.gz_offset = 12;

	MPU9250.mx_scale = 1.042146;
	MPU9250.my_scale = 0.906667;
	MPU9250.mz_scale = 1.066667;

	MPU9250.mx_offset = 18;
	MPU9250.my_offset = -7;
	MPU9250.mz_offset = -28;

	//configuring mpu6500 sensor//
	//get device who am i ID first and check if its the standard value
	mpu_i2c_readregister(WHO_AM_I_MPU9250, &i2c_rx_data);
	if(i2c_rx_data != WHO_AM_I_MPU9250_ANS)
	{
		return mpu6500_who_am_I_fault;
	}

	//reset mpu9250
	i2c_tx_data = 0x80;
	mpu_i2c_writeregister(PWR_MGMT_1, &i2c_tx_data);
	HAL_Delay(10);

	//Enable Pass through mode to access AK8963 on I2C bus from stm32
	i2c_tx_data = 0x02;
	mpu_i2c_writeregister(INT_PIN_CFG, &i2c_tx_data);

	// setting accel range to 16G as default
	mpu_set_accel_FSR(ACCEL_RANGE_16G);

	// setting the gyro range to 2000DPS as default
	mpu_set_gyro_FSR(GYRO_RANGE_2000DPS);

	// setting bandwidth to 99Hz
	mpu_set_accel_bandwidth(ACCEL_DLPF_BANDWIDTH_45HZ);

	// setting gyro bandwidth to 92Hz
	mpu_set_gyro_bandwidth(GYRO_DLPF_BANDWIDTH_41HZ);

	//set sample rate divider to 200Hz
	i2c_tx_data = 0x04;
	mpu_i2c_writeregister(SMPLRT_DIV, &i2c_tx_data);

	//enable raw data ready interrupt
	i2c_tx_data = 0x01;
	mpu_i2c_writeregister(INT_ENABLE, &i2c_tx_data);

	//select clock automatically as PLL or internal oscillator
	i2c_tx_data = 0x01;
	mpu_i2c_writeregister(PWR_MGMT_1, &i2c_tx_data);
	HAL_Delay(10);

	//configuring AK8963 sensor//
	//check who am I value of the AK8963
	ak_i2c_readregister(AK8963_WHO_AM_I, &i2c_rx_data);
	if(i2c_rx_data != AK8963_WHO_AM_I_ANS)
	{
		return ak8963_who_am_I_fault;
	}

	//reset the AK8963
	i2c_tx_data = 0x01;
	ak_i2c_writeregister(AK8963_CNTL2,&i2c_tx_data);
	HAL_Delay(10);

	//select 16 bit data output mode and select fused ROM access mode
	i2c_tx_data = 0x1F;
	ak_i2c_writeregister(AK8963_CNTL1,&i2c_tx_data);
	HAL_Delay(10);

	//read fused ROM data and put data in global data structure
	read_ak_fuseROM_data();

	//select power down mode before entering the continuous measurement mode
	i2c_tx_data = 0x16;
	ak_i2c_writeregister(AK8963_CNTL1,&i2c_tx_data);
	HAL_Delay(10);

	if(MAG_CALIB == true)
	{
		calibrate_compass();
	}

	if(GYRO_CALIB == true)
	{
		calibrate_gyro();
	}

	HAL_TIM_Base_Start_IT(&htim2);
	return 0;
}

/*
 * Function: calibrate_compass
 * ----------------------------
 *   Perform calibration of the ak8963 compass. Sensor should be manually
 *   rotated in all directions for the calibration to be effective.
 *
 *   returns: void
 */
void calibrate_compass()
{
	int16_t mx_min = 32760;                      //raw data extremes
	int16_t my_min = 32760;
	int16_t mz_min = 32760;
	int16_t mx_max = -32760;
	int16_t my_max = -32760;
	int16_t mz_max = -32760;

	float chord_x,  chord_y,  chord_z;            // Used for calculating scale factors
	float chord_average;

	for(uint16_t i = 0; i<3000; i++)
	{
		ak8963_i2c_readst1register();
		while(i2c1_xfer_in_progress != false){}
		if(MPU9250.ak8963_st1 & AK8963_DRDY_Mask)
		{
			ak_i2c_readdataregisters();
			while(i2c1_xfer_in_progress != false){}
			if(!(MPU9250.ak_raw_data_buffer[6] & AK8963_OVF_Mask))
			{
				// Combine LSB,MSB, apply ASA corrections
				MPU9250.mx = (MPU9250.ak_raw_data_buffer[0] | (int16_t)(MPU9250.ak_raw_data_buffer[1] << 8)) * MPU9250.ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
				MPU9250.my = (MPU9250.ak_raw_data_buffer[2] | (int16_t)(MPU9250.ak_raw_data_buffer[3] << 8)) * MPU9250.ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
				MPU9250.mz = (MPU9250.ak_raw_data_buffer[4] | (int16_t)(MPU9250.ak_raw_data_buffer[5] << 8)) * MPU9250.ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections

				//find min max
				mx_min = min(MPU9250.mx, mx_min);
				mx_max = max(MPU9250.mx, mx_max);
				my_min = min(MPU9250.my, my_min);
				my_max = max(MPU9250.my, my_max);
				mz_min = min(MPU9250.mz, mz_min);
				mz_max = max(MPU9250.mz, mz_max);
			}
			HAL_Delay(10);
		}
		else if(!(MPU9250.ak8963_st1 & AK8963_DRDY_Mask))
		{
			i--;
		}
	}

	// ----- Calculate hard-iron offsets
	MPU9250.mx_offset = (mx_max + mx_min) / 2;                     // Get average magnetic bias in counts
	MPU9250.my_offset = (my_max + my_min) / 2;
	MPU9250.mz_offset = (mz_max + mz_min) / 2;

	// ----- Calculate soft-iron scale factors
	chord_x = ((float)(mx_max - mx_min)) / 2;                 // Get average max chord length in counts
	chord_y = ((float)(my_max - my_min)) / 2;
	chord_z = ((float)(mz_max - mz_min)) / 2;

	chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

	MPU9250.mx_scale = chord_average / chord_x;                          // Calculate X scale factor
	MPU9250.my_scale = chord_average / chord_y;                          // Calculate Y scale factor
	MPU9250.mz_scale = chord_average / chord_z;                          // Calculate Z scale factor

	__NOP();
}

/*
 * Function: calibrate_compass
 * ----------------------------
 *   Perform calibration of the Gyroscope in mpu6500.
 *
 *   returns: void
 */
void calibrate_gyro()
{
	uint16_t counts = (uint16_t)(((float)1/(float)(MPU9250.gyro_calibration_delay/1000.00)))*10;

	for(int gyro_data_count = 0; gyro_data_count < counts; gyro_data_count++)
	{
		mpu_i2c_readdataregisters();

		MPU9250.mpu_combined_bytes_data[3] = (int16_t)(MPU9250.mpu_raw_data_buffer[8]  << 8) | MPU9250.mpu_raw_data_buffer[9];
		MPU9250.mpu_combined_bytes_data[4] = (int16_t)(MPU9250.mpu_raw_data_buffer[10] << 8) | MPU9250.mpu_raw_data_buffer[11];
		MPU9250.mpu_combined_bytes_data[5] = (int16_t)(MPU9250.mpu_raw_data_buffer[12] << 8) | MPU9250.mpu_raw_data_buffer[13];

		MPU9250.gx_cal_sum += MPU9250.mpu_combined_bytes_data[3];
		MPU9250.gy_cal_sum += MPU9250.mpu_combined_bytes_data[4];
		MPU9250.gz_cal_sum += MPU9250.mpu_combined_bytes_data[5];

		HAL_Delay(MPU9250.gyro_calibration_delay);
	}

	MPU9250.gx_offset = (int16_t)(MPU9250.gx_cal_sum/counts);
	MPU9250.gy_offset = (int16_t)(MPU9250.gy_cal_sum/counts);
	MPU9250.gz_offset = (int16_t)(MPU9250.gz_cal_sum/counts);

	MPU9250.gyro_calibration_done = 1;
}

/*
 * Function: get_mpu_data
 * ----------------------------
 *   Get accelerometer, gyroscope and temperature data from the mpu6500
 *
 *   returns: status of the function. If the timer is not up yet it returns busy.
 *   		  if the timer is up then the timer flag is only cleared if data read and processed.
 *   		  First a DMA transfer is initiated and returns HAL_OK then after the transfer is completed the data is processed.
 *   		  And returns HAL_OK again.
 */
HAL_StatusTypeDef get_mpu_data()
{
	if(read_mpu_data_flag != true) //check if 5ms has passed since last data collection from sensor
	{
		//inform that 5ms has not passed
		return HAL_BUSY;
	}
	//5ms has passed since last data collection from mpu6500

	//read new data from the sensor if not already done
	if(mpu6500_drdy_flag == false && mpu6500_data_read_started == false && i2c1_xfer_in_progress == false)//
	{
		mpu6500_data_read_started = true;
		mpu_i2c_readdataregisters();
		return HAL_OK;
	}
	else if(mpu6500_drdy_flag == true)
	{

		MPU9250.mpu_combined_bytes_data[0] = (int16_t)(MPU9250.mpu_raw_data_buffer[0] << 8) | MPU9250.mpu_raw_data_buffer[1];
		MPU9250.mpu_combined_bytes_data[1] = (int16_t)(MPU9250.mpu_raw_data_buffer[2] << 8) | MPU9250.mpu_raw_data_buffer[3];
		MPU9250.mpu_combined_bytes_data[2] = (int16_t)(MPU9250.mpu_raw_data_buffer[4] << 8) | MPU9250.mpu_raw_data_buffer[5];
		MPU9250.mpu_combined_bytes_data[3] = ((int16_t)(MPU9250.mpu_raw_data_buffer[8] << 8)  | MPU9250.mpu_raw_data_buffer[9]) - MPU9250.gx_offset;
		MPU9250.mpu_combined_bytes_data[4] = ((int16_t)(MPU9250.mpu_raw_data_buffer[10] << 8) | MPU9250.mpu_raw_data_buffer[11]) - MPU9250.gy_offset;
		MPU9250.mpu_combined_bytes_data[5] = ((int16_t)(MPU9250.mpu_raw_data_buffer[12] << 8) | MPU9250.mpu_raw_data_buffer[13]) - MPU9250.gz_offset;

		//transform, remove bias and scale the readings to standard units. ax, ay and az are in m/s2. gx, gy and gz are in rad/s.
//		MPU9250.ax = (tX[0]*MPU9250.mpu_combined_bytes_data[0] + tX[1]*MPU9250.mpu_combined_bytes_data[1] + tX[2]*MPU9250.mpu_combined_bytes_data[2])*MPU9250.accel_scale_factor;
//		MPU9250.ay = (tY[0]*MPU9250.mpu_combined_bytes_data[0] + tY[1]*MPU9250.mpu_combined_bytes_data[1] + tY[2]*MPU9250.mpu_combined_bytes_data[2])*MPU9250.accel_scale_factor;
//		MPU9250.az = (tZ[0]*MPU9250.mpu_combined_bytes_data[0] + tZ[1]*MPU9250.mpu_combined_bytes_data[1] + tZ[2]*MPU9250.mpu_combined_bytes_data[2])*MPU9250.accel_scale_factor;
//		MPU9250.gx = (tX[0]*MPU9250.mpu_combined_bytes_data[3] + tX[1]*MPU9250.mpu_combined_bytes_data[4] + tX[2]*MPU9250.mpu_combined_bytes_data[5])*MPU9250.gyro_scale_factor;
//		MPU9250.gy = (tY[0]*MPU9250.mpu_combined_bytes_data[3] + tY[1]*MPU9250.mpu_combined_bytes_data[4] + tY[2]*MPU9250.mpu_combined_bytes_data[5])*MPU9250.gyro_scale_factor;
//		MPU9250.gz = (tZ[0]*MPU9250.mpu_combined_bytes_data[3] + tZ[1]*MPU9250.mpu_combined_bytes_data[4] + tZ[2]*MPU9250.mpu_combined_bytes_data[5])*MPU9250.gyro_scale_factor;



		MPU9250.ax = MPU9250.mpu_combined_bytes_data[0]*MPU9250.accel_scale_factor;
		MPU9250.ay = MPU9250.mpu_combined_bytes_data[1]*MPU9250.accel_scale_factor;
		MPU9250.az = MPU9250.mpu_combined_bytes_data[2]*MPU9250.accel_scale_factor;
		MPU9250.gx = MPU9250.mpu_combined_bytes_data[3]*MPU9250.gyro_scale_factor;
		MPU9250.gy = MPU9250.mpu_combined_bytes_data[4]*MPU9250.gyro_scale_factor;
		MPU9250.gz = MPU9250.mpu_combined_bytes_data[5]*MPU9250.gyro_scale_factor;


//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // for logic analyzer debugging.
		mpu6500_data_read_started = false;
		read_mpu_data_flag = false;
		mpu6500_drdy_flag = false;
		return HAL_OK;
	}
	return HAL_BUSY;
}

/*
 * Function: get_ak_data
 * ----------------------------
 *   Get compass data from ak8963
 *
 *   returns: status of the function. If the timer is not up yet it returns busy.
 *   		  if the timer is up then the timer flag is only cleared if data is read and processed.
 *   		  First a DMA transfer is initiated to read the status register. In second run of this function the
 *   		  another DMA transfer is initiated to read data from the data registers if DRDY bit of ak8963 is set.
 *   		  And returns HAL_OK.
 */
HAL_StatusTypeDef get_ak_data()
{
	if(read_ak_data_flag != true)
	{
		return HAL_BUSY;
	}

	if(ak8963_strdy_flag == false && i2c1_xfer_in_progress == false && ak8963_status_read_started == false)
	{
		ak8963_status_read_started = true;
		ak8963_i2c_readst1register();
	}
	else if(ak8963_strdy_flag == true && ak8963_data_read_started == false && i2c1_xfer_in_progress == false )
	{
		ak8963_status_read_started = false;
		ak8963_data_read_started = true;
		ak_i2c_readdataregisters();
	}
	else if(ak8963_drdy_flag == true)
	{
		ak8963_data_read_started = false;

		if(MPU9250.ak_raw_data_buffer[6] & AK8963_OVF_Mask)
		{
			read_ak_data_flag = false;
			ak8963_strdy_flag = false;
			ak8963_drdy_flag = false;
			return HAL_ERROR;
		}

			MPU9250.mx = (MPU9250.ak_raw_data_buffer[0] | (int16_t)(MPU9250.ak_raw_data_buffer[1] << 8)) * MPU9250.ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
			MPU9250.my = (MPU9250.ak_raw_data_buffer[2] | (int16_t)(MPU9250.ak_raw_data_buffer[3] << 8)) * MPU9250.ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
			MPU9250.mz = (MPU9250.ak_raw_data_buffer[4] | (int16_t)(MPU9250.ak_raw_data_buffer[5] << 8)) * MPU9250.ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections

			MPU9250.mx = (MPU9250.mx - MPU9250.mx_offset) * MPU9250.mx_scale;
			MPU9250.my = (MPU9250.my - MPU9250.my_offset) * MPU9250.my_scale;
			MPU9250.mz = (MPU9250.mz - MPU9250.mz_offset) * MPU9250.mz_scale;

			ak8963_strdy_flag = false;
			ak8963_drdy_flag = false;
			read_ak_data_flag = false;
			return HAL_OK;
	}
	return HAL_BUSY;

}

/*
 * Function: mpu_i2c_readdataregisters
 * ----------------------------
 *   Read data registers from the mpu6500
 *
 *   returns: void
 *
 */
void mpu_i2c_readdataregisters()
{
	i2c1_xfer_in_progress = true;
	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, MEM_ADD_SIZE, MPU9250.mpu_raw_data_buffer, MPU_ALL_DATA_SIZE);
}

/*
 * Function: ak_i2c_readdataregisters
 * ----------------------------
 *   Read data registers from the ak8963
 *
 *   returns: void
 *
 */
void ak_i2c_readdataregisters()
{
	i2c1_xfer_in_progress = true;
	HAL_I2C_Mem_Read_DMA(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, MEM_ADD_SIZE, MPU9250.ak_raw_data_buffer, 7);
}

/*
 * Function: ak8963_i2c_readst1register
 * ----------------------------
 *   Read status1 register data from the ak8963
 *
 *   returns: void
 *
 */
void ak8963_i2c_readst1register()
{
	i2c1_xfer_in_progress = true;
	HAL_I2C_Mem_Read_DMA(&hi2c1, AK8963_ADDRESS, AK8963_ST1, MEM_ADD_SIZE, &MPU9250.ak8963_st1, 1);
}

/*
 * Function: read_ak_fuseROM_data
 * ----------------------------
 *   Read fuseROM register data from the ak8963. And converts and write to
 *   MPU9250 data structure.
 *
 *   returns: void
 *
 */
void read_ak_fuseROM_data()
{
	i2c1_rx_cplt = false;
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Mem_Read_DMA(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, MEM_ADD_SIZE, MPU9250.raw_ASA, 3);
	while(i2c1_rx_cplt != true){}


	MPU9250.ASAX = ((MPU9250.raw_ASA[0] - (uint8_t)128)/256.0f + 1.0f)* 4912.0f / 32760.0f;                       // Adjust data
	MPU9250.ASAY = ((MPU9250.raw_ASA[1] - (uint8_t)128)/256.0f + 1.0f)* 4912.0f / 32760.0f;
	MPU9250.ASAZ = ((MPU9250.raw_ASA[2] - (uint8_t)128)/256.0f + 1.0f)* 4912.0f / 32760.0f;
}

/*
 * Function: mpu_set_gyro_FSR
 * ----------------------------
 *   Takes the desired gyroscope FSR and set it in the register location of the mpu6500.
 *
 *   gyro_fsr: The full scale range desired to be set for gyroscope.
 *
 *   returns: void
 *
 */
void mpu_set_gyro_FSR(GyroFSR gyro_fsr)
{
	uint8_t gyro_config_reg_val;
	mpu_i2c_readregister(GYRO_CONFIG, &gyro_config_reg_val);

	switch(gyro_fsr) {
	  case GYRO_RANGE_250DPS:
		  MPU9250.gyro_scale_factor = (1.0f/131.0f) * D2R;
		  gyro_config_reg_val |= 0x00;
		  mpu_i2c_writeregister(GYRO_CONFIG,&gyro_config_reg_val);
	    break;
	  case GYRO_RANGE_500DPS:
		  MPU9250.gyro_scale_factor = (1.0f/65.5f) * D2R;
		  gyro_config_reg_val |= 0x08;
		  mpu_i2c_writeregister(GYRO_CONFIG,&gyro_config_reg_val);
	    break;
	  case GYRO_RANGE_1000DPS:
		  MPU9250.gyro_scale_factor = (1.0f/32.8f) * D2R;
		  gyro_config_reg_val |= 0x10;
		  mpu_i2c_writeregister(GYRO_CONFIG,&gyro_config_reg_val);
	    break;
	  case GYRO_RANGE_2000DPS:
		  MPU9250.gyro_scale_factor = (1.0f/16.4f) * D2R;
		  gyro_config_reg_val |= 0x18;
		  mpu_i2c_writeregister(GYRO_CONFIG,&gyro_config_reg_val);
	    break;

	  default:
	    break;
	}
}

/*
 * Function: mpu_set_accel_FSR
 * ----------------------------
 *   Takes the desired accelerometer FSR and set it in the register location of the mpu6500.
 *
 *   accel_fsr: The full scale range desired to be set for accelerometer.
 *
 *   returns: void
 *
 */
void mpu_set_accel_FSR(GyroFSR accel_fsr)
{
	uint8_t accel_config_reg_val;
	mpu_i2c_readregister(ACCEL_CONFIG, &accel_config_reg_val);

	switch(accel_fsr) {
	  case ACCEL_RANGE_2G:
		  MPU9250.accel_scale_factor = (1.0f/16384.0f) * G2MSS;
		  accel_config_reg_val |= 0x00;
		  mpu_i2c_writeregister(ACCEL_CONFIG,&accel_config_reg_val);
	    break;
	  case ACCEL_RANGE_4G:
		  MPU9250.accel_scale_factor = (1.0f/8192.0f) * G2MSS;
		  accel_config_reg_val |= 0x08;
		  mpu_i2c_writeregister(ACCEL_CONFIG,&accel_config_reg_val);
	    break;
	  case ACCEL_RANGE_8G:
		  MPU9250.accel_scale_factor = (1.0f/4096.0f) * G2MSS;
		  accel_config_reg_val |= 0x10;
		  mpu_i2c_writeregister(ACCEL_CONFIG,&accel_config_reg_val);
	    break;
	  case ACCEL_RANGE_16G:
		  MPU9250.accel_scale_factor = (1.0f/2048.0f) * G2MSS;
		  accel_config_reg_val |= 0x18;
		  mpu_i2c_writeregister(ACCEL_CONFIG,&accel_config_reg_val);
	    break;

	  default:
	    break;
	}
}

/*
 * Function: mpu_set_gyro_bandwidth
 * ----------------------------
 *   Takes the desired bandwidth of DLPF and set it in the register location of the mpu6500.
 *
 *   gyro_bw: The bandwidth desired to be set for gyroscope.
 *
 *   returns: void
 *
 */
void mpu_set_gyro_bandwidth(GYRODLPFBandwidth gyro_bw)
{
	uint8_t config_reg_val;
	mpu_i2c_readregister(CONFIG, &config_reg_val);

	switch(gyro_bw) {
	  case GYRO_DLPF_BANDWIDTH_250HZ:
		  MPU9250.gyro_calibration_delay = 4;
		  config_reg_val |= 0x00;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_184HZ:
		  MPU9250.gyro_calibration_delay = 6;
		  config_reg_val |= 0x01;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_92HZ:
		  MPU9250.gyro_calibration_delay = 11;
		  config_reg_val |= 0x02;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_41HZ:
		  MPU9250.gyro_calibration_delay = 25;
		  config_reg_val |= 0x03;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_20HZ:
		  MPU9250.gyro_calibration_delay = 50;
		  config_reg_val |= 0x04;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_10HZ:
		  MPU9250.gyro_calibration_delay = 100;
		  config_reg_val |= 0x05;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_5HZ:
		  MPU9250.gyro_calibration_delay = 200;
		  config_reg_val |= 0x06;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;
	  case GYRO_DLPF_BANDWIDTH_3600HZ:
		  config_reg_val |= 0x07;
		  mpu_i2c_writeregister(CONFIG,&config_reg_val);
	    break;

	  default:
	    break;
	}
}

/*
 * Function: mpu_set_accel_bandwidth
 * ----------------------------
 *   Takes the desired bandwidth of DLPF and set it in the register location of the mpu6500.
 *
 *   accel_bw: The bandwidth desired to be set for accelerometer.
 *
 *   returns: void
 *
 */
void mpu_set_accel_bandwidth(ACCELDLPFBandwidth accel_bw)
{
	uint8_t accel_config2_reg_val;
	mpu_i2c_readregister(ACCEL_CONFIG2, &accel_config2_reg_val);

	switch(accel_bw) {
	  case ACCEL_DLPF_BANDWIDTH_218HZ:
		  accel_config2_reg_val |= 0x01;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_184HZ:
		  accel_config2_reg_val |= 0x02;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_99HZ:
		  accel_config2_reg_val |= 0x03;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_45HZ:
		  accel_config2_reg_val |= 0x04;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_10HZ:
		  accel_config2_reg_val |= 0x05;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_5HZ:
		  accel_config2_reg_val |= 0x06;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;
	  case ACCEL_DLPF_BANDWIDTH_420HZ:
		  accel_config2_reg_val |= 0x07;
		  mpu_i2c_writeregister(ACCEL_CONFIG2,&accel_config2_reg_val);
	    break;

	  default:
	    break;
	}
}

/*
 * Function: HAL_I2C_MemRxCpltCallback
 * ----------------------------
 *   Handles the interrupts which are fired at completion of data read operations.
 *
 *   hi2c: pointer to the data structure of the I2C handle.
 *
 *   returns: void
 *
 */
void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if(hi2c == &hi2c1)
	{

		i2c1_rx_cplt = true; //variable only used for the startup initialization phase.


		if(i2c1_xfer_in_progress == true)
		{
			i2c1_xfer_in_progress = false;
		}

		if(mpu6500_data_read_started == true)
		{
			mpu6500_drdy_flag = true;
		}

		if(ak8963_status_read_started == true && (MPU9250.ak8963_st1 & AK8963_DRDY_Mask))
		{
			ak8963_strdy_flag = true;
		}
		else if(ak8963_status_read_started == true)
		{
			ak8963_status_read_started = false;
		}

		if(ak8963_data_read_started == true)
		{
			ak8963_drdy_flag = true;
		}
	}
}

/*
 * Function: HAL_I2C_MemTxCpltCallback
 * ----------------------------
 *   Handles the interrupts which are fired at completion of data write operations.
 *
 *   hi2c: pointer to the data structure of the I2C handle.
 *
 *   returns: void
 *
 */
void HAL_I2C_MemTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if(hi2c == &hi2c1)
	{
		i2c1_tx_cplt = true;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)
	{
		read_mpu_data_flag = true;
	}
}
