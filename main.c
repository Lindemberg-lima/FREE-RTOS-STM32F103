/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN Author */
/**
 * @author  Lindemberg Roberto de Lima
 * @brief   Application main file
 *
 * @details
 * Application-level code including RTOS task design, sensor integration,
 * data processing, and UART transmission logic were developed by the author.
 *
 * Auto-generated initialization code was produced by STM32CubeMX
 * and is licensed by STMicroelectronics.
 */
/* USER CODE END Author */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "qmc5883p.h"
#include "bmp280.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId Bmp280TaskHandle;
osThreadId UartTaskHandle;
osThreadId Qmc5883pTaskHandle;
osThreadId Mpu6050TaskHandle;

/* USER CODE BEGIN PV */
BMP280_Handle_t bmp280;

static QueueHandle_t bmp280_queue;
static QueueHandle_t mpu6050_queue;
static QueueHandle_t qmc5883p_queue;

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE END 5 */
void StartBmp280Task(void const * argument);
void StartUartTask(void const * argument);
void StartQmc5883pTask(void const * argument);
void StartMpu6050Task(void const * argument);
void StartKalmanTask(void const * argument);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  bmp280_queue = xQueueCreate(5, sizeof(SensorData_t));
  mpu6050_queue = xQueueCreate(12, sizeof(readingImu));
  qmc5883p_queue = xQueueCreate(5, sizeof(QMC5883P_Data_t));

  osThreadDef(UartTask, StartUartTask, osPriorityBelowNormal, 0, 256);
        UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  osThreadDef(Mpu6050Task, StartMpu6050Task, osPriorityLow, 0, 256);
      Mpu6050TaskHandle = osThreadCreate(osThread(Mpu6050Task), NULL);

  osThreadDef(Bmp280Task, StartBmp280Task, osPriorityNormal, 0, 256);
    Bmp280TaskHandle = osThreadCreate(osThread(Bmp280Task), NULL);

  osThreadDef(Qmc5883pTask, StartQmc5883pTask, osPriorityBelowNormal, 0, 256);
    Qmc5883pTaskHandle = osThreadCreate(osThread(Qmc5883pTask), NULL);

  /* INICA O ESCALONADOR*/
  osKernelStart();

  while (1) {}
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN Header_StartMpu6050Task */
/**
  * @brief  Function implementing the Mpu6050Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMpu6050Task */
void StartMpu6050Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	readingImu  convertedImu;
	readingAcel readingA;
	readingGyro readingG;
	MPU6050_Init(&hi2c1);

		 for (;;)
	        {
			 MPU6050_ReadAccel(&hi2c1, &readingA);
			 MPU6050_ReadGyro(&hi2c1,&readingG);

			 float accelG[3] = {readingA.accelX / 16384.0, readingA.accelY / 16384.0, readingA.accelZ / 16384.0};
			 float gyroGpS[3] = {readingG.gyroX / 131.0, readingG.gyroY / 131.0, readingG.gyroZ / 131.0};

			 convertedImu.imugyroX=accelG[0];
			 convertedImu.imugyroY=accelG[1];
			 convertedImu.imugyroZ=accelG[2];
			 convertedImu.imuaccelX=gyroGpS[0];
			 convertedImu.imuaccelY=gyroGpS[1];
			 convertedImu.imuaccelZ=gyroGpS[2];

			 xQueueSend(mpu6050_queue,&convertedImu,0);
			     /* USER CODE BEGIN 3 */
			 osDelay(250);
	        }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBmp280Task */

/**
* @brief Function implementing the Bmp280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmp280Task */
void StartBmp280Task(void const * argument)
{
  /* USER CODE BEGIN StartBmp280Task */
  /* Infinite loop */
	     SensorData_t bmpData;
		 bmpData.temperature = 0;
		 bmpData.pressure    = 0;
		 /* Inicializa BMP280 */
		 bmp280.hi2c = &hi2c1;
		 float temp, press;
	     if (BMP280_Init(&bmp280) != HAL_OK)
	        {
			   Error_Handler();
	        }

		 for (;;)
			  {
			   if (BMP280_ReadTempPressure(&bmp280, &temp, &press) == HAL_OK)
			      {
			         bmpData.temperature = temp;
			         bmpData.pressure    = press;
			         xQueueSend(bmp280_queue,&bmpData,0);
		           }

			   osDelay(250);
			   }
  /* USER CODE END StartBmp280Task */
}

/* USER CODE BEGIN Header_StartQmc5883pTask */
/**
* @brief Function implementing the Qmc5883pTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartQmc5883pTask */
void StartQmc5883pTask(void const * argument)
{
  /* USER CODE BEGIN StartQmc5883pTask */
  /* Infinite loop */
	QMC5883P_Data_t mag;

	  if (QMC5883P_Init(&hi2c1) != HAL_OK)
	      {
	      Error_Handler();   // sensor não respondeu
	      }

	for(;;)
       {
		if (QMC5883P_Read(&mag) == HAL_OK)
		   {

			xQueueSend(qmc5883p_queue,&mag,0);
	// float yaw=atan2(My,Mx);
	    	}
	    osDelay(1);

        }
  /* USER CODE END StartQmc5883pTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */

	    readingImu imuUart;
		imuUart.imugyroX=0;
		imuUart.imugyroY=0;
		imuUart.imugyroZ=0;
		imuUart.imuaccelX=0;
		imuUart.imuaccelY=0;
		imuUart.imuaccelZ=0;

		SensorData_t uartData;
		uartData.temperature=0;
		uartData.pressure=0;

		QMC5883P_Data_t magUart;
		magUart.x=0;
		magUart.y=0;
		magUart.z=0;

		static char txBuffer[128];

	    for (;;)
	    {

	                if (xQueueReceive(bmp280_queue,&uartData,portMAX_DELAY) == pdPASS)
	        		{
	                 snprintf(txBuffer,
	    	         sizeof(txBuffer),
	    	         "%.2f,%.2f,",
	    			 uartData.temperature,
	    			 uartData.pressure);
	        		}
	                HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,strlen(txBuffer),50);
	                osDelay(1);

	                if (xQueueReceive(qmc5883p_queue,&magUart,portMAX_DELAY) == pdPASS)
	               	 {
	               	  snprintf(txBuffer,
	               	  sizeof(txBuffer),
	               	  "%.2d, %.2d, %.2d,",
					  magUart.x,
					  magUart.y,
					  magUart.z);
	               	  }
	               	  HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,strlen(txBuffer),50);
	                osDelay(1);

	               	if (xQueueReceive(mpu6050_queue,&imuUart,portMAX_DELAY) == pdPASS)
	               		{
	               		 snprintf(txBuffer,sizeof(txBuffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
	               		 imuUart.imugyroX,
	               		 imuUart.imugyroY,
	               		 imuUart.imugyroZ,
	               		 imuUart.imuaccelX,
	               		 imuUart.imuaccelY,
	               		 imuUart.imuaccelZ);
	               		 }
	                 HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,strlen(txBuffer),50);
	    }

  /* USER CODE END StartUartTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

