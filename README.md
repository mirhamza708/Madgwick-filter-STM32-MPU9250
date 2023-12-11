# Madgwick-filter-STM32-MPU9250
This project implements the open source Madgwick filter on STM32 platforms using the MPU9250 IMU.
The visual results of the filter are shown in the following short video. youtube link: https://youtu.be/vXpDsQi6GR4


https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/25eeeb81-becc-441f-8b4b-8bd5b3dca93c



There are multiple ways to estimate the orientation of an object in 3D space, the most easiest 
way is to attach a gyroscope onto the object and measure the rotation rates of the object and integrate
those measurements every 0.01 seconds or so, this will give a crude orientation of the object which will
keep drifting away from the actual orientation of the object. This is because the sensor data we are integrating
is noisy and the noise is also integrated and thus causing drift. 
The second method is to use an accelerometer which will be a much better option because it has no drift component.
the only two reasons the accelerometer alone can not be used are that the accelerometer readings are too much noisy and
we cannot measure the yaw angle with just an accelerometer. If let's say we want to use it on a Quadcopter
the vibrations from the propellers amplifies the noise and make it unusable.
The same kind of problems also come with the magnetometer. The magnetometer readings will be distorted as soon as it comes 
near to an external magnetic field.
This problem was solved by Sebastian Madgwick in his PhD thesis. More info on the algorithm can be
found here https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ .
This github repository https://github.com/xioTechnologies/Fusion/tree/main contains more information on the filter and
also provides the code for implementation in C and Python.

The algorithm uses the Gyroscope readings as a base and correct the drift component with the accelerometer and magnetometer.
This is not the only estimation method, there are other algorithms such as Kalman filter, Extended Kalman filter and Uncsented Kalman Filter
but the madgwick filter is much more light weight in terms of computations. This link https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
provides more information on the computational load of the Madgwick filter. 

In this project I have implemeted the Madgwick filter on STM32F401 using MPU9250. The STM32 runs at 84Mhz.
The MPU9250 is a combination of gyroscope, accelerometer and a magnetometer.
I2C communication protocol is used to read data from all three sensors. 
Reading data from the MPU9250 is a bit complex because the gyroscope and accelerometer are placed on a seperate die than the magnetomer die,
as stated by invensense the MPU9250 is basically an MPU6500(gyroscope and accelerometer) and AK8963(magnetomer). To read data from MPU6500
we simply read the data registers of the sensor but to read data from AK8963 we have two options, one is to use the MPU6500 to get data from
AK8963 and place that data in the external sensor data registers or the second option is to put the AK8963 on the main I2C bus. In this project
the second option is used. The MPU6500 is configured such that it provides data at 200Hz and the AK8963 provides data at 100Hz. DMA transfer is used to transfer data on the I2C bus.
A timer is configured to interrupt the microcontroller every 1 millisecond. In the timer interrupt callback 3 counters variables are incremented which are
ak_read_counter, uart_send_data_counter and filter_counter. When the ak_read_counter reaches a value of 10 (providing 10ms delay between readings)
the counter is reset to zero and read_ak_data_flag is set, this flag is read in the MPU9250.c file and data is requested from the AK8963 sensor
and the flag is reset only when data has been read and processed.
In the same manner the other two counter are incremented and reset providing a filter execution rate of 500Hz and
50Hz data sending rate to PC via USB virtual COM port. The MPU6500 has a raw data ready interrupt which is set every 200Hz(defined by the Sample rate divider)
when data is ready to be read from the sensor. This interrupt is used in STM32 to set read_mpu_data_flag flag to inform the main loop that data is ready to be read, this flag is reset when
data has been processed.
By doing this we have data available from MPU6500 at 200Hz, from AK8963 at 100Hz for the Madgwick filter which runs at 500Hz and the final result is sent to PC at 50Hz. Using DMA allows 
for sending and receiving data on I2C bus without the processor involvement.

The image below shows the I2C transactions at runtime. 14 bytes are transferred from AK8963 and 17 bytes from MPU6500.
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/ccd4f4e0-ec80-4474-8dc8-f6fbf46f7c3c)
The green measurement is for the AK8963 data and the red measurement is for the MPU6500 data.

# Setting up STM32 environment
## Setting clock
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/e74a471a-ba5b-434f-8691-af013d98b68a)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/2f50e901-f4c4-48a5-bcd9-cd70eb40d6f5)

## Setting up timer
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/95adcbec-5841-4da2-a0a2-0a489afef01e)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/3eafb6a8-f754-4874-9066-9f714febaa79)


## Setting up the I2C Master with DMA
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/92ae1285-d0a3-4cca-aeef-ec1bae6baffa)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/445e88f2-ed4c-4e9d-a526-c6c92c77b23e)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/2e28fd89-b1fa-4ca7-8d05-201e1abf3d50)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/27550bc3-7ed0-4288-98fe-ba88b156344b)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/ffb50c57-223d-4b50-9306-01c9a39f7c38)

## Setting up the External interrupt for Raw data ready interrupt from MPU6500
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/4aa38b17-f21e-4bb4-b75f-f637f0c87c1a)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/790e0fd9-88ad-41dd-8b7b-42d9cf92733c)

## Setting up USB Virtual COM Port
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/3a5303b7-836b-422d-90f6-2c487cfb331e)
![image](https://github.com/mirhamza708/Madgwick-filter-STM32-MPU9250/assets/55946600/55c3f387-e8ea-46b4-991d-cee0090db4b1)

# Main code
In main first the MPU9250 is initialized like this 
```
MPU9250_init();
```

Then in the loop two functions are called to read data from sensors
```
get_mpu_data();
get_ak_data();
```

After the above two functions the execute_filter_flag flag is checked if its true then the filter is executed and the flag reset.
```
if(execute_filter_flag == true)
{
  execute_filter_flag = false;
  MadgwickAHRSupdate(MPU9250.gx, MPU9250.gy, MPU9250.gz, MPU9250.ax, MPU9250.ay, MPU9250.az, MPU9250.my, MPU9250.mx, -MPU9250.mz);
  computeAngles(); //used to convert Quaternions to Euler angles
}
```

then we check the uart_send_data_flag flag and transmit data onto PC like this
```
if(uart_send_data_flag == true)
{
  uart_send_data_flag = false;
  length = sprintf((char*)UART_tx_buffer, "Orientation: %f %f %f\r\n", yaw, pitch, roll);
  CDC_Transmit_FS(UART_tx_buffer, length);
}
```

# Reported Issues
Only one issue have been experienced up till now which is related to the I2C bus communication. When the MCU is constantly receiving data
from the sensors, now let's say a START condition has been sent on the bus and some data has been requested from one of the sensor if at this moment the 
MCU is programmed from the IDE the sensor holds the I2C bus busy by keeping the SDA line low.
To recover from this error condition a power reset followed by the MCU reset throught the RESET key is required. 


The example shows the implementation and should work without any changes if the above settings have been done correctly.
I have tried to comment every line and function which needed explaination. 
Good luck and enjoy.
