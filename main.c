#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
extern int timee;

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

uint16_t CCR3_Val = 620;
uint16_t CCR4_Val = 623;
uint16_t PrescalerValue = 0;
TIM_OCInitTypeDef  TIM_OCInitStructure;

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

void init_tim2()
{	
	//RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(7200-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(100-1);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
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
	float acc[3],gyro[3];
	float theta;
	float error;
	float derivative;
	float setpoint = atanf(-0.09/1.09);
	float Kp= 5.0;
	float Kd= 0.0;
	init_led();
	init_usart1();
	init_tim2();
	init_tim4_pwm();
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	if( MPU6050_TestConnection() == TRUE)
	{
	   puts("connection success\r\n");
	}else {
	   puts("connection failed\r\n");
	}
	//printf("test float%f\r\n",num);
	while (1) {
		//puts("running now\r\n");
		MPU6050_GetRawAccelGyro(buff);
		for ( int i = 0; i<3; i++)
			acc[i] = (buff[i]/16384.0);
		for ( int i = 0; i<3; i++)
			gyro[i] = (buff[i+2]/131.0);

		theta = atanf(acc[0]/acc[2])*180/3.14;
		error = setpoint - theta;
		derivative = gyro[1];

		CCR3_Val = CCR3_Val+(Kp*error+Kd*derivative);
		CCR4_Val = CCR4_Val-(Kp*error+Kd*derivative);
		TIM4->CCR3 = CCR3_Val;
		TIM4->CCR4 = CCR4_Val;
		if  (CCR3_Val>1240){

			CCR3_Val = 620;

        } else if (CCR3_Val < 0){

            CCR3_Val = 620;

        }


        if(CCR4_Val > 1246){

            CCR4_Val = 623;
        
        } else if (CCR4_Val < 0){

      
            CCR4_Val = 623;

        }

        setpoint=theta;

        gpio_toggle(GPIOA, GPIO_Pin_0);
        gpio_toggle(GPIOA, GPIO_Pin_1);
        delay_gg(1);

	}
}
