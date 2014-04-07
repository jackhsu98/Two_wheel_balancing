#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
int16_t GYRO_X_OFFSET;
int16_t GYRO_Y_OFFSET;
int16_t GYRO_Z_OFFSET;
int16_t ACC_X_OFFSET;
int16_t ACC_Y_OFFSET;
int16_t ACC_Z_OFFSET;
/*simple moving average*/
int16_t sma_filter_int16(int16_t new_data, int16_t * fifo, uint16_t n)
{	
	int16_t i = 0;
	int32_t sum = 0;;
	for ( i=1 ; i<n ; i++)
		fifo[i] = fifo[i-1];
	fifo[0] = new_data;
	for ( i=0 ; i<n ; i++)
		sum += fifo[i];
	return (int16_t)(sum/n);
}
void imu_calibration()
{	
	#define FILTER_SIZE 200
	int16_t buff[6];
	int16_t acc_fifo[3][FILTER_SIZE];
	int16_t gyro_fifo[3][FILTER_SIZE];
	int16_t count = FILTER_SIZE*5;
	while( count--)
	{
		MPU6050_GetRawAccelGyro(buff);
		ACC_X_OFFSET = sma_filter_int16(buff[0], acc_fifo[0], FILTER_SIZE);
	 	ACC_Y_OFFSET = sma_filter_int16(buff[1], acc_fifo[1], FILTER_SIZE);
	 	ACC_Z_OFFSET = sma_filter_int16(buff[2], acc_fifo[2], FILTER_SIZE);
		GYRO_X_OFFSET = sma_filter_int16(buff[3], gyro_fifo[0], FILTER_SIZE);
		GYRO_Y_OFFSET = sma_filter_int16(buff[4], gyro_fifo[1], FILTER_SIZE);
		GYRO_Z_OFFSET = sma_filter_int16(buff[5], gyro_fifo[2], FILTER_SIZE);
		//delay_ms(1);

	}
	ACC_Z_OFFSET += -16384;

	
}