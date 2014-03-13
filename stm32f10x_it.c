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
#include "stm32f10x_it.h"
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "MPU6050.h"
#include <math.h>

extern uint16_t CCR3_Val;
extern uint16_t CCR4_Val;
extern uint16_t PrescalerValue;

extern float theta;
extern float error;
extern float derivative;
extern float setpoint;
extern float Kp;
extern float Kd;
#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679
//#include "led.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
  int timee=0.0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void TIM2_IRQHandler()
{
            
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
                timee++;
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
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

void SysTick_Handler(void)
{
    int16_t buff[6];
    float acc[3],gyro[3];
    setpoint= atanf(-0.09/1.09);
    Kp= 5.0;
    Kd= 0.0;
    MPU6050_GetRawAccelGyro(buff);
    for ( int i = 0; i<3; i++)
      acc[i] = (buff[i]/16384.0);
    for ( int i = 0; i<3; i++)
      gyro[i] = (buff[i+2]/131.0);

    theta = atanf(acc[0]/acc[2])*180/PI;
    error = setpoint - theta;
    derivative = gyro[1];

    CCR3_Val = CCR3_Val-(Kp*error+Kd*derivative);
    CCR4_Val = CCR4_Val+(Kp*error+Kd*derivative);
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
