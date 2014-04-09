#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
extern int timee;

//#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

extern uint16_t CCR3_Val;
extern uint16_t CCR4_Val;
uint16_t PrescalerValue = 0;
TIM_OCInitTypeDef  TIM_OCInitStructure;

extern float pitch;
extern float pitch1;
extern float error1;
extern float derivative1;
extern float derivative2;
extern int16_t buff[6];
extern float acc[3],gyro[3];
extern float setpoint_v;
extern float Kp;
extern float Kd;
extern float I;
extern float I_yaw;
extern float AX, AZ,ax, az;


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
	TIM_TimeBaseStructure.TIM_Period = 57600 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 25 -1;
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
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(20-1);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void init_tim3()
{	
	//RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(7200-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(20-1);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
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
	imu_calibration();
	init_tim3();
	//printf("test float%f\r\n",num);
	while (1) {
		
		 
		 // printf("Pitch1: %f", pitch1 );
        // printf("Pitch: %f\t", pitch );
	    printf("Error1: %f\t", error1 );
	    printf("I: %f\t", I );
	    printf("I_yaw: %f\t", I_yaw );
	    // printf("Derivative1: %f\r\n", derivative1 );
		// printf("Derivative2: %f\t", derivative2 );
        printf("TIM4->CCR3: %d\t",TIM4->CCR3 );
        printf("TIM4->CCR4: %d\t",TIM4->CCR4 );
        printf("CCR3: %d\t", CCR3_Val );
		printf("CCR4: %d\r\n", CCR4_Val);
		
        delay_gg(10);

	}
}
