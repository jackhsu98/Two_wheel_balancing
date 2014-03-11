#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
extern timee;

uint16_t CCR3_Val = 720;
uint16_t CCR4_Val = 720;
uint16_t PrescalerValue = 0;
TIM_OCInitTypeDef  TIM_OCInitStructure;
void delay(uint32_t delay_count)
{
	while (delay_count) delay_count--;
}

void init_tim4_pwm()
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	

	/* TIM3 clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  	/* GPIOA and GPIOB clock enable */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);	

  	/*GPIOB Configuration: TIM3 channel1, 2, 3 and 4 */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_Init(GPIOB, &GPIO_InitStructure);

 
  	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 200 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel2 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	  TIM_ARRPreloadConfig(TIM4, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM4, ENABLE);
}
void init_led()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA0 and PA1 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->ODR ^= GPIO_Pin;
}
void delay_gg(uint32_t count){
    timee=0;
	while(count>timee){}
		
}

int main(void)
{
	int16_t buff[6];
	float acc[3],gyro[3],num=3.4444;
	float theta;
	float phi;
	float error;
	float derivative;
	float dt = 0.1;
	float previous_error = 0.0;
	float setpoint = 0.0;
	float Kp=100.0;
	float Kd=10.0;
	init_led();
	init_usart1();
	init_tim4_pwm();
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	if( MPU6050_TestConnection() == TRUE)
	{
	   puts("connection success\r\n");
	}else {
	   puts("connection failed\r\n");
	}
	printf("test float%f\r\n",num);
	while (1) {
		//puts("running now\r\n");
		MPU6050_GetRawAccelGyro(buff);
		for ( int i = 0; i<3; i++)
			acc[i] = (buff[i]/16384.0);
		for ( int i = 0; i<3; i++)
			gyro[i] = (buff[i+2]/131.0);

		theta = 2*atanf(acc[0]/acc[2]);
		phi = atanf(gyro[2]/gyro[0]);
		//printf("Theta,%f\r\n",theta);
		error = setpoint - phi;
		printf("Error,%f\r\n",error);
		derivative = (error - previous_error)/dt;
		printf("Derivative,%f\r\n",derivative); 

		CCR3_Val = CCR3_Val+(Kp*error+Kd*derivative);
		CCR4_Val = CCR4_Val-(Kp*error+Kd*derivative);
		printf("CCR3_Val,%f\r\n",CCR3_Val);
		printf("CCR4_Val,%f\r\n",CCR4_Val);
		TIM4->CCR3 = CCR3_Val;
		TIM4->CCR4 = CCR4_Val;
		if ( (CCR3_Val>7200) || (CCR4_Val > 7200) ){

			CCR3_Val = 720;
			CCR4_Val = 720;

		} else if ( (CCR3_Val < 200) || (CCR4_Val < 200) ){

			CCR3_Val = 720;
			CCR4_Val = 720;

		}
		previous_error = error;

		gpio_toggle(GPIOA, GPIO_Pin_0);
		gpio_toggle(GPIOA, GPIO_Pin_1);
		delay_gg(0.1);

	}
}
