/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>

extern int16_t GYRO_X_OFFSET;
extern int16_t GYRO_Y_OFFSET;
extern int16_t GYRO_Z_OFFSET;
extern int16_t ACC_X_OFFSET;
extern int16_t ACC_Y_OFFSET;
extern int16_t ACC_Z_OFFSET;

uint16_t CCR3_Val = 4910;
uint16_t CCR4_Val = 4935;

float ax,ay,az,gyro_x,gyro_y,gyro_z;
float pitch;
float pitch1;
float error1;
float derivative1;
float derivative2;
float I = 0.0;
float I_yaw = 0.0;
int16_t buff[6];
float acc[3],gyro[3];
float setpoint_v=0;
float Kp = 90.0;//100.0;
float Ki = 0.0;//40.0;
float Kd = 100.0;//30.0;
float Kp_yaw = 5.0;//10.0;
float Ki_yaw = 20.0;
float AX = 0.0;
float AZ = 0.0;
float apha = 0.1;


#define PI 3.1415926535
#define R2D 57.29577951
#define D2R 0.01745
//#include "led.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
int timee=0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void TIM2_IRQHandler()
{
            
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
                timee=timee+1;
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}

void boundary(float Max, float min, float *Target)
{
  if (*Target>Max)
  {
    *Target = Max;
  } else if (*Target<min)
  {
    *Target = min;
  } 
}

void TIM3_IRQHandler()
{
            
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){     
          //printf("test float%f\r\n",num);
          
          //puts("running now\r\n");
          MPU6050_GetRawAccelGyro(buff);
          for ( int i = 0; i<3; i++)
            acc[i] = buff[i];
          for ( int i = 0; i<3; i++)
            gyro[i] = buff[i+3];
          
          ax = (acc[0]-ACC_X_OFFSET)/16384.0;
          ay = (acc[1]-ACC_Y_OFFSET)/16384.0;
          az = (acc[2]-ACC_Z_OFFSET)/16384.0;
          gyro_x = (gyro[0]-GYRO_X_OFFSET)/131.0;
          gyro_y = (gyro[1]-GYRO_Y_OFFSET)/131.0;
          gyro_z = (gyro[2]-GYRO_Z_OFFSET)/131.0;

          AX = AX*(1-apha)+ax*apha;
          AZ = AZ*(1-apha)+az*apha;

          pitch = atanf(ax/az)*R2D;
          pitch1 = atanf(AX/AZ)*R2D;

          derivative1 = gyro_y;
          derivative2 = gyro_z;

          error1 = setpoint_v - pitch1;
          I = I + error1*0.002;
          I_yaw =I_yaw + derivative2*0.002;

          boundary(30,-30,&I);

          // if (I>30)
          // {
          //   I = 30;
          // } else if (I<-30)
          // {
          //   I = -30;
          // }
          
           if (I_yaw>30)
          {
            I_yaw = 30;
          } else if (I_yaw<-30)
          {
            I_yaw = -30;
          }


          CCR3_Val = 4910-(int16_t)(Kp*error1+Ki*I-Kd*derivative1)-(int16_t)(Kp_yaw*derivative2+Ki_yaw*I_yaw);
          CCR4_Val = 4935+(int16_t)(Kp*error1+Ki*I-Kd*derivative1)-(int16_t)(Kp_yaw*derivative2+Ki_yaw*I_yaw);
          TIM4->CCR3 = CCR3_Val;
          TIM4->CCR4 = CCR4_Val;

          if  (CCR3_Val>9819){
            CCR3_Val = 9819;
          } else if (CCR3_Val < 1){
            CCR3_Val = 1;
          }

          if(CCR4_Val > 9869){
            CCR4_Val = 9869;
          } else if (CCR4_Val < 1){
            CCR4_Val = 1;
          }

          

          gpio_toggle(GPIOA, GPIO_Pin_0);
          gpio_toggle(GPIOA, GPIO_Pin_1);
          TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
          
        }
}
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
