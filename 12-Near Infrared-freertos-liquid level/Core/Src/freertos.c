/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PWR_ON_OFF.h"
#include "Water_tank_liquid.h"
#include "string.h"
#include "stdio.h"
#include "flash_write_read.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LED_TaskHandle;
osThreadId PWR_TaskHandle;
osThreadId BOOT_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_LED_Task(void const * argument);
void Start_PWR_Task(void const * argument);
void Start_BOOT_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, Start_LED_Task, osPriorityIdle, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of PWR_Task */
  osThreadDef(PWR_Task, Start_PWR_Task, osPriorityNormal, 0, 128);
  PWR_TaskHandle = osThreadCreate(osThread(PWR_Task), NULL);

  /* definition and creation of BOOT_Task */
  osThreadDef(BOOT_Task, Start_BOOT_Task, osPriorityBelowNormal, 0, 128);
  BOOT_TaskHandle = osThreadCreate(osThread(BOOT_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_LED_Task */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_LED_Task */
void Start_LED_Task(void const * argument)
{
  /* USER CODE BEGIN Start_LED_Task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
//		STMFLASH_Read(0x0807000,(uint32_t*)flag,1);
//		printf("water_flag_fill_flag读取到的值是： %d",flag);
	  osDelay(500);
	  
  }
  /* USER CODE END Start_LED_Task */
}

/* USER CODE BEGIN Header_Start_PWR_Task */
/**
* @brief Function implementing the PWR_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PWR_Task */
void Start_PWR_Task(void const * argument)
{
  /* USER CODE BEGIN Start_PWR_Task */
  /* Infinite loop */
  for(;;)
  {
	  PWR_ON_OFF();
	  osDelay(20);
  }
  /* USER CODE END Start_PWR_Task */
}

/* USER CODE BEGIN Header_Start_BOOT_Task */
/**
* @brief Function implementing the BOOT_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_BOOT_Task */
void Start_BOOT_Task(void const * argument)
{
  /* USER CODE BEGIN Start_BOOT_Task */
  /* Infinite loop */
  uint8_t water_low_flag = 0;//液位高低的标志位
  uint8_t water_high_flag = 0;//液位高低的标志位
  uint8_t water_flag_fill = 0;//液位满的标志位，高低液位均达标了。
	water_flag_fill = water_low_flag + water_high_flag;
	
  for(;;)
  {
	  
	  water_low_flag = Water_tank_liquid_low(0);
	  water_high_flag = Water_tank_liquid_high(0);
//	  if(water_low_flag == 1)//低液位的标志位
//	  {  
//		  HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
////		  water_flag = 0;
//	  }
//	  if(water_high_flag == 2)//高液位的标志位
//	  {
//		  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
////		  water_flag = 0;
//	  }
	  
	  if(water_flag_fill_flag == 0)
	  {
		  
			if(water_low_flag  == 1)//低液位有效
			{
				HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);//低液位的指示灯亮起

			}   
			 if( water_high_flag  == 2)//高液位有效
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);//低液位的指示灯亮起

			}   
			
			 if(water_low_flag + water_high_flag  == 0)//没有达到高液位和低液位，指示灯全灭。
			{
				HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);

			}   
			  
			else if(water_low_flag + water_high_flag  == 3)
			{
			  water_flag_fill_flag = 1;
			  STMFLASH_Write(FLASH_ADDR,(uint32_t *)&water_flag_fill_flag,1);//存入此刻液位的高低的标志位。
			  for(int i = 0;i<6;i++)//蜂鸣器响三声后，执行软件复位。
			  {
				  HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin);
				  osDelay(500);    
			  }      					
			  HAL_NVIC_SystemReset();//	  执行软件复位。

			}   
	  }
	  
	  
	  else if(water_flag_fill_flag == 1)
	  {
		  
		  if(water_high_flag + water_low_flag < 3)//高液位的标志位
		  {
			  water_flag_fill_flag = 0;
			  STMFLASH_Write(FLASH_ADDR,(uint32_t *)&water_flag_fill_flag,1);
			  HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	//		  water_flag = 0;
		  }
	  
	  
	  }
	  
	  printf("此刻water_low_flag的值是： %d\r\n",water_flag_fill_flag);

	  
//	  printf("water_low_flag的值是：%d\r\n",water_low_flag);
//	  printf("water_high_flag的值是：%d\r\n",water_high_flag);
	  osDelay(100);
  }
  /* USER CODE END Start_BOOT_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
