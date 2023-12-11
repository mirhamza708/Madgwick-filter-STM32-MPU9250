/*
 * MPU9250.h
 *
 *  Created on: Nov 29, 2023
 *      Author: Hamza
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

extern I2C_HandleTypeDef hi2c1;
extern struct _MPU9250 MPU9250;

typedef enum { false, true } bool;

#define MAG_CALIB	false
#define GYRO_CALIB  false

// Constants
#define RAD2DEG 57.2957795131
#define G2MSS	9.81
// Defines
#define AK8963_ADDRESS   0x0C<<1
#define MPU9250_ADDRESS  0x68<<1  // Device address when ADO = 0

#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_WHO_AM_I_ANS  0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2	 0x0B  // soft reset write 0x01 to this register
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define AK8963_DRDY_Mask 0x01
#define AK8963_OVF_Mask  0x08

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define WHO_AM_I_MPU9250_ANS 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


#define CS_SELECT         0
#define CS_DESELECT       1
#define SPI_TIMOUT_MS     100
#define MEM_ADD_SIZE 	  1
#define REG_MEM_SIZE	  1
#define MPU_ALL_DATA_SIZE 14
#define AK_ALL_DATA_SIZE  6

#define D2R 0.01745329

typedef enum mpuinit_error_codes_
{
	mpu6500_who_am_I_fault = 0,
	ak8963_who_am_I_fault,
} mpuinit_error_codes;

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroFSR;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelFSR;

typedef enum ACCELDLPFBandwidth_ {
	ACCEL_DLPF_BANDWIDTH_218HZ,
	ACCEL_DLPF_BANDWIDTH_184HZ,
	ACCEL_DLPF_BANDWIDTH_99HZ,
	ACCEL_DLPF_BANDWIDTH_45HZ,
	ACCEL_DLPF_BANDWIDTH_10HZ,
	ACCEL_DLPF_BANDWIDTH_5HZ,
	ACCEL_DLPF_BANDWIDTH_420HZ
} ACCELDLPFBandwidth;

typedef enum GYRODLPFBandwidth_ {
	GYRO_DLPF_BANDWIDTH_250HZ = 0,
	GYRO_DLPF_BANDWIDTH_184HZ,
	GYRO_DLPF_BANDWIDTH_92HZ,
	GYRO_DLPF_BANDWIDTH_41HZ,
	GYRO_DLPF_BANDWIDTH_20HZ,
	GYRO_DLPF_BANDWIDTH_10HZ,
	GYRO_DLPF_BANDWIDTH_5HZ,
	GYRO_DLPF_BANDWIDTH_3600HZ
} GYRODLPFBandwidth;


typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;

typedef struct MAGCalibration_ {
	int16_t mag_x_offset;
	int16_t mag_y_offset;
	int16_t mag_z_offset;

	int16_t mag_x_scale;
	int16_t mag_y_scale;
	int16_t mag_z_scale;

}MAGCalibration;

typedef struct GYROCalibration_ {
	int16_t gyro_x_bias;
	int16_t gyro_y_bias;
	int16_t gyro_z_bias;
}GYROCalibration;

struct _MPU9250{
	uint8_t first_start;
	uint8_t gyro_calibration_done;
	uint8_t gyro_calibration_delay;
	float accel_scale_factor;
	float gyro_scale_factor;
	int gx_cal_sum;
	int gy_cal_sum;
	int gz_cal_sum;
	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
	uint8_t mpu_raw_data_buffer[14];
	uint8_t ak_raw_data_buffer[7];
	int16_t mpu_combined_bytes_data[6];
	int16_t ak_combined_bytes_data[6];
	uint8_t ak8963_st1;
	float ASAX;
	float ASAY;
	float ASAZ;
	uint8_t raw_ASA[3];
	float mx_scale;
	float my_scale;
	float mz_scale;
	int16_t mx_offset;
	int16_t my_offset;
	int16_t mz_offset;
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	float mx;
	float my;
	float mz;
};

void mpu_i2c_writeregister(uint8_t address, uint8_t * data);
void mpu_i2c_readregister(uint8_t address, uint8_t * data);
void ak_i2c_writeregister(uint8_t address, uint8_t * data);
void ak_i2c_readregister(uint8_t address, uint8_t * data);
mpuinit_error_codes MPU9250_init();
void calibrate_compass();
void calibrate_gyro();
HAL_StatusTypeDef get_mpu_data();
HAL_StatusTypeDef get_ak_data();
void mpu_i2c_readdataregisters();
void ak_i2c_readdataregisters();
void ak8963_i2c_readst1register();
void read_ak_fuseROM_data();
void mpu_set_gyro_FSR(GyroFSR gyro_fsr);
void mpu_set_accel_FSR(GyroFSR accel_fsr);
void mpu_set_gyro_bandwidth(GYRODLPFBandwidth gyro_bw);
void mpu_set_accel_bandwidth(ACCELDLPFBandwidth accel_bw);
void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef * hi2c);
void HAL_I2C_MemTxCpltCallback (I2C_HandleTypeDef * hi2c);

#endif /* INC_MPU9250_H_ */
