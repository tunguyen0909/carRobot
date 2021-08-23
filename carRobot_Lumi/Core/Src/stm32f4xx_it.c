/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include <string.h>
#include <stdio.h>
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define sampling_Time 0.025
#define inv_sampling_Time 40

extern char dataReceiveJetson[6];
extern char dataReceive[8];
volatile int vanToc1 = 0;
volatile int vanToc2 = 0;
volatile int vanToc3 = 0;
volatile int vanToc4 = 0;
volatile int count = 0;
int dem=0;
int goc_RX = 0;

volatile int pluse1 = 0;
volatile int pluse_pre1 = 0;
volatile int rSpeed1 = 0, Err1, pre_Err1, Kp1 = 11, Ki1 = 2, Kd1 = 5;
volatile int pPart1 =0, dPart1=0, iPart1 = 0;
volatile int Output1=0;

volatile int pluse2 = 0;
volatile int pluse_pre2 = 0;
volatile int rSpeed2 = 0 , Err2, pre_Err2, Kp2 = 10, Ki2 = 0, Kd2 = 5;
volatile int pPart2 =0, dPart2=0, iPart2 = 0;
volatile int Output2=0;

volatile int pluse3 = 0;
volatile int pluse_pre3 = 0;
volatile int rSpeed3 = 0, Err3, pre_Err3, Kp3 = 11, Ki3 = 1, Kd3 = 4;
volatile int pPart3 =0, dPart3=0, iPart3 = 0;
volatile int Output3=0;

volatile int pluse4 = 0;
volatile int pluse_pre4 = 0;
volatile int rSpeed4 = 0, Err4, pre_Err4, Kp4 = 11, Ki4 = 1 , Kd4 = 5;
volatile int pPart4 =0, dPart4=0, iPart4 = 0;
volatile int Output4=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN EV */
int triTuyeDoi(int a)
{
	if(a < 0)
		a = -a;
	else 
		a = a;
	return a;
}

int chuyen(char c)
{
		return (int)c-48;
}
void getVanToc(char data[])
{
			vanToc1 = chuyen(data[0])*10 + chuyen(data[1]);
			
			goc_RX = (chuyen(data[2])*10 + chuyen(data[3]))*10 + chuyen(data[4]);
			
			vanToc2 = chuyen(data[2])*10 + chuyen(data[3]);
			vanToc3 = chuyen(data[4])*10 + chuyen(data[5]);
			vanToc4 = chuyen(data[6])*10 + chuyen(data[7]);
	
}

void PID_control_DC1(int speed_des1)
{
	rSpeed1 = pluse1 - pluse_pre1;
	pluse_pre1 = pluse1;
	Err1 = speed_des1 - triTuyeDoi(rSpeed1);
	 
	pPart1 = Kp1*(Err1);
	dPart1 = Kd1*(Err1 - pre_Err1)*inv_sampling_Time;
	iPart1 += Ki1*sampling_Time*Err1;
	Output1 += pPart1 + dPart1 + iPart1;
	
	if(Output1 > 4000)
		Output1 = 4000-1;
	if(Output1 <= 0)
		Output1 = 1;
	
	pre_Err1 = Err1;
}

void PID_control_DC2(int speed_des2)
{
	rSpeed2 = pluse2 - pluse_pre2;
	pluse_pre2 = pluse2;
	Err2 = speed_des2 - triTuyeDoi(rSpeed2);
	 
	pPart2 = Kp2*(Err2);
	dPart2 = Kd2*(Err2 - pre_Err2)*inv_sampling_Time;
	iPart2 += Ki2*sampling_Time*Err2;
	Output2 += pPart2 + dPart2 + iPart2;
	
	if(Output2 > 4000)
		Output2 = 4000-1;
	if(Output2 <= 0)
		Output2 = 1;
	
	pre_Err2 = Err2;
}

void PID_control_DC3(int speed_des3)
{
	rSpeed3 = pluse3 - pluse_pre3;
	pluse_pre3 = pluse3;
	Err3 = speed_des3 - triTuyeDoi(rSpeed3);
	 
	pPart3 = Kp3*(Err3);
	dPart3 = Kd3*(Err3 - pre_Err3)*inv_sampling_Time;
	iPart3 += Ki3*sampling_Time*Err3;
	Output3 += pPart3 + dPart3 + iPart3;
	
	if(Output3 > 4000)
		Output3 = 4000-1;
	if(Output3 <= 0)
		Output3 = 1;
	
	pre_Err3 = Err3;
}

void PID_control_DC4(int speed_des4)
{
	rSpeed4 = pluse4 - pluse_pre4;
	pluse_pre4 = pluse4;
	Err4 = speed_des4 - triTuyeDoi(rSpeed4);
	 
	pPart4 = Kp4*(Err4);
	dPart4 = Kd4*(Err4 - pre_Err4)*inv_sampling_Time;
	iPart4 += Ki4*sampling_Time*Err4;
	Output4 += pPart4 + dPart4 + iPart4;
	
	if(Output4 > 4000)
		Output4 = 4000-1;
	if(Output4 <= 0)
		Output4 = 1;
	
	pre_Err4 = Err4;
}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == 0)
	{
		pluse1 = pluse1 + 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == 1)
	{
		pluse1 = pluse1 - 1;
	}
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 0)
	{
		pluse2 = pluse2 + 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 1)
	{
		pluse2 = pluse2 - 1;
	}
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == 0)
	{
		pluse3 = pluse3 + 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == 1)
	{
		pluse3 = pluse3 - 1;
	}
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 0)
	{
		pluse4 = pluse4 + 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 1)
	{
		pluse4 = pluse4 - 1;
	}
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	getVanToc(dataReceiveJetson);
	PID_control_DC1(vanToc1);
	PID_control_DC2(vanToc1);
	PID_control_DC3(vanToc1);
	PID_control_DC4(vanToc1);
	count++;
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
