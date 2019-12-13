
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include "dwt_delay.h"
#define INIT_STATE 0
#define FW_STATE 1
#define CHECKRIGHT 2
#define CHECKLEFT 3
#define ADJUST_POSITION 4
#define GREET_HUMAN 5
#define FW_NOTTRUE 6
#define RAMPSTATE 7

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId StateMachineHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
 
	uint8_t forwardcommand[] = {0xCA,0x32,0xC2,0x32};
	uint8_t stopcommand[] = {0xCA,0x00,0xC2,0x00,0xC1,0x00,0xC9,0x00};
	uint8_t AdjustRight[] = {0xCA, 0x20, 0xC2, 0x27};
	uint8_t AdjustLeft[] = {0xCA, 0x27, 0xC2 , 0x20};
	uint8_t Turnleft[] = {0xCA,0x20,0xC1,0x20};
	uint8_t Turnright[] = {0xC2,0x20,0xC9,0x20};
	uint8_t test[] = {0xCA,0x25,0xC2,0x25};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void FSM(void const * argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 300);
	return ch;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	int distance;
	int distanceL;
	int distanceR;
	int flag_got_it = 0;
	extern int angle;

	static int DistanceCounter = 0;
	int state = INIT_STATE;
	static int InitAngle;
	int Global_Counter =0;


	int abs(int a)
	{
		if(a < 0)
			return a*(-1);
		else
			return a;
	}
		
	void MoveForward(void)
	{
		HAL_UART_Transmit(&huart1,forwardcommand,sizeof(forwardcommand),250);
	}

	void MoveRight(void)
	{
		HAL_UART_Transmit(&huart1,Turnright,sizeof(Turnright),250);
	}
	void MoveLeft(void)
	{
		HAL_UART_Transmit(&huart1,Turnleft,sizeof(Turnleft),250);
	}
	void Stop(void)
	{
			HAL_UART_Transmit(&huart1,stopcommand,sizeof(stopcommand),250);
	}
	
	void AdjustingRight(void)
	{
				HAL_UART_Transmit(&huart1,AdjustRight,sizeof(AdjustRight),250);
	}

	void AdjustingLeft(void)
	{
				HAL_UART_Transmit(&huart1,AdjustLeft,sizeof(AdjustLeft),250);
	}
	void write(void)
	{
		HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin,GPIO_PIN_RESET);
		DWT_Delay(2);
		HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin,GPIO_PIN_SET);
		DWT_Delay(10);
		HAL_GPIO_WritePin(Trigger_GPIO_Port,Trigger_Pin,GPIO_PIN_RESET);
	}
		
		
	void MoveForwardE(void)
	{
		if((angle-InitAngle) < 0)
		{
			test[1] = 0x22;
			test[3] = 0x30;
		}
		else if((angle-InitAngle) > 0)
		{
			test[1] = 0x30;
			test[3] = 0x22;
		}
		else
		{
			test[1] = 0x30;
			test[3] = 0x30;
		}
		
		HAL_UART_Transmit(&huart1,test,sizeof(test),250);
		printf("Current speed : %i , %i \r\n", test[1],test[3]);

	}
	
	int Read(void)
	{
		int count =0;
		while (!HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin));
		do                                                                                   
		{
			count ++;
			DWT_Delay(1);
		}
		while (HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin));
		return (count/58);
	}


	int Read_sensorR(void)
	{
		int count =0;
		while(!HAL_GPIO_ReadPin(EchoR_GPIO_Port,EchoR_Pin));
		do
		{
			count ++;
			DWT_Delay(1);
		}
		while (HAL_GPIO_ReadPin(EchoR_GPIO_Port,EchoR_Pin));
		return (count/58);
		
	}
	int Read_sensorL(void)
	{
		int count =0;
		while(!HAL_GPIO_ReadPin(EchoL_GPIO_Port,EchoL_Pin));
		do
		{
			count ++;
			DWT_Delay(1);
		}
		while (HAL_GPIO_ReadPin(EchoL_GPIO_Port,EchoL_Pin));
		return (count/58);
		
	}

	int CalculateDifference(int s)
	{
		 return angle - s;
	}

	void AdjustAngle(int tem1)
	{
		while(1)
		{
			printf("Angle : %i , LowerRange:  %i, UpperRange: %i \r\n", angle, tem1-2, tem1+2);
			if((angle > tem1-3) && ( angle < tem1 + 3))
			{
					Stop();
					HAL_Delay(1000);
					break;
			}
		}
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of StateMachine */
  osThreadDef(StateMachine, FSM, osPriorityAboveNormal, 0, 128);
  StateMachineHandle = osThreadCreate(osThread(StateMachine), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* USER CODE BEGIN RTOS_THREADS */

//										
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	

  while (1)
  {	}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Echo_Pin EchoL_Pin */
  GPIO_InitStruct.Pin = Echo_Pin|EchoL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EchoR_Pin */
  GPIO_InitStruct.Pin = EchoR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EchoR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_FSM */
/**
  * @brief  Function implementing the StateMachine thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_FSM */
void FSM(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	//MoveForward();
  for(;;)
  { 
	
			switch(state)
			{	
				case INIT_STATE:
					while(!flag_got_it)
					{printf("FLAG_GOT_IT %i \r\n",flag_got_it);}
          InitAngle = angle;
					printf("InitAngle : %i \r\n", InitAngle);
					state = FW_STATE;
					break;
				
				
				case FW_STATE:
					printf("FW\r\n");
					write();
					distance  = Read();
					printf("Distance FW : %i\r\n", distance);
					if (distance >= 4)
					{
						printf("Error Val = %i \r\n",(angle-InitAngle));
						MoveForwardE();
						printf("IN FW Distance Moved %i:\r\n", DistanceCounter);
						DistanceCounter++;
						if(DistanceCounter >= 3750)
							{
								state = RAMPSTATE;
								break;
							}
						if(DistanceCounter%1500 == 0)
							InitAngle += 2;
					}
					else 
					{
						Stop();
						HAL_Delay(100);
						state = CHECKRIGHT;
					}
				break;
				
				
				case CHECKRIGHT:
						printf("CHEKCRIGHT \r\n");
						write();
						distanceR = Read_sensorR();
						printf("Distance R %i \r\n", distanceR);

						if (distanceR < 4) // Right side is blocked
									state = CHECKLEFT;
						else if( distanceR >= 4)
						{
							printf("Distance R %i \r\n", distanceR);
							int tem1 = (angle + 60) %360;
							MoveRight();
							AdjustAngle(tem1);
							Stop();
							HAL_Delay(100);
							
							state = FW_NOTTRUE;

								
						}
				break;
					
				case CHECKLEFT:
						printf("CHECKLEFT \r\n");
						write();
						distanceL = Read_sensorL();
						if (distanceL < 4) // Right side is blocked
									state = FW_STATE;
						else if(distanceL >= 4)
						{
							
							int tem1 = (angle + 300) %360;
							MoveLeft();
							AdjustAngle(tem1);
							Stop();
							HAL_Delay(100);
							state = FW_NOTTRUE;
								
						}
				break;
				
				case FW_NOTTRUE:
					Global_Counter = 0;
					printf("FWNOTTRUE \r\n");
					write();
					distance = Read();
					printf("Distance FWNOTTRUE : %i \r\n", distance);
					if(distance >= 4)
					{
						MoveForward();
						state = FW_NOTTRUE;
					}
					else
						{
							MoveLeft();
							AdjustAngle(InitAngle);
							state = FW_STATE;
						}
					break;
						
				case RAMPSTATE:
					Stop();
					break;
				
				default:
					printf("default \r\n");
					state = FW_STATE;
				break;
			}
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
