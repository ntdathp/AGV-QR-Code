/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "pid.h"
#include "utils.h"
#include "motor.h"
#include "../MPU6050/mpu6050.h"
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

/* USER CODE BEGIN PV */
#define SAMPLE_STRING "0360,0045,0045,1,0"
#define ARRAY_CHECKER "1111,1111,1111,1,1"
#define MAX_LEN_DATA 18
void SerialInit(void);
void SerialAcceptReceive(void);


void MotorMovePos(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl1, PID_CONTROL_t *tPIDControl2,Motor_t *tmotor1, Motor_t *tmotor2, uint8_t dir1, uint8_t dir2);

#define GO_AHEAD 1
#define GO_BACK 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char dataBuffer[MAX_LEN_DATA];

MPU6050_t MPU6050;

uint32_t g_nCmdPulse;
float g_dCmdVel;

uint32_t nPulse1;
uint32_t nPulse2;

uint8_t tProcess;
ArrData_t arrData1;

uint8_t dir1, dir2;
char statusOK[] = "OK\r\n";
char result[100];

int32_t dutyCycle_global_1 = 0,  dutyCycle_global_2 = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  while (MPU6050_Init(&hi2c1) == 1);
  SerialInit();
  MotorInit();
  MotorSetRun();

  tProcess = NONE;


  tMotor1.ptd = 0.091; //2700
  tMotor2.ptd = 0.091; //2580


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
void SerialInit(void)
{
  HAL_UART_Receive_IT(&huart2, (uint8_t *)dataBuffer, MAX_LEN_DATA);
}

// receive data
void SerialAcceptReceive(void)
{
  HAL_UART_Receive_IT(&huart2, (uint8_t *)dataBuffer, MAX_LEN_DATA);
}

// interupt uart RX
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart2.Instance)
  {

    	__HAL_TIM_SetCounter(&htim3, 0);
	    __HAL_TIM_SetCounter(&htim4, 0);
	    tProcess = NONE;
	    PIDReset(&tPID_1);
	    PIDReset(&tPID_2);


	    if(!strcmp(dataBuffer, ARRAY_CHECKER)){
	    	HAL_UART_Transmit(&huart2, (uint8_t *)statusOK, sizeof(statusOK), 1000);
	    }
	    else {
	        HAL_UART_Transmit(&huart2, (uint8_t *)dataBuffer, MAX_LEN_DATA, 1000);
	    	    arrData1 = ArrProcess(dataBuffer);
	    	    MotorTrapzoidalInit(&tProfile, arrData1.pos1, arrData1.vel1, arrData1.acc1);
	    	    dir1 = arrData1.dir1;
	    	    dir2 = arrData1.dir2;

	    	    tProcess = RUN_TEST;

	    	        if (dir1 == HEAD)
	    	        {
	    	        	Motor1Forward();
	    	        	tMotor1.dir = HEAD;
	    	        }
	    	        else
	    	        {
	    	        	Motor1Backward();
	    	        	tMotor1.dir = BACK;
	    	        }
	    	        if (dir2 == HEAD)
	    	        {
	    	        	Motor2Forward();
	    	        	tMotor2.dir = HEAD;
	    	        }
	    	        else
	    	        {
	    	        	Motor2Backward();
	    	        	tMotor2.dir = BACK;
	    	        }
	    }
	    SerialAcceptReceive();
  }
}


/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == htim2.Instance)
  {
    switch (tProcess)
    {
    case NONE:
    	MPU6050_Read_All(&hi2c1, &MPU6050);
    	break;
    case RUN_TEST:
    	MotorSetRun();
    	ReadEncoder(&tMotor1, &htim4);
    	ReadEncoder(&tMotor2, &htim3);
    	MPU6050_Read_All(&hi2c1, &MPU6050);
    	MotorMovePos(&tProfile, &tPID_1, &tPID_2, &tMotor1, &tMotor2, dir1, dir2);
    }
  }
}

void MotorMovePos(PROFILE_t *tProfile, PID_CONTROL_t *tPIDControl1, PID_CONTROL_t *tPIDControl2,Motor_t *tmotor1, Motor_t *tmotor2, uint8_t dir1, uint8_t dir2)
{
  int32_t g_nDutyCycle_1, g_nDutyCycle_2;


  float dPosTemp = 0;

  // Profile trapezoidal Speed
  if (tProfile->nTime <= tProfile->dMidStep1)
  {
    dPosTemp = (int32_t)(tProfile->dA1 * tProfile->nTime * tProfile->nTime);
    g_dCmdVel = 2 * tProfile->dA1 * tProfile->nTime;
  }
  else if (tProfile->nTime <= tProfile->dMidStep2)
  {
    dPosTemp = (int32_t)(tProfile->dA2 * tProfile->nTime + tProfile->dB2);
    g_dCmdVel = tProfile->dA2;
  }
  else if (tProfile->nTime <= tProfile->dMidStep3)
  {
    dPosTemp = (int32_t)(tProfile->dA3 * tProfile->nTime * tProfile->nTime + tProfile->dB3 * tProfile->nTime + tProfile->dC3);
    g_dCmdVel = 2 * tProfile->dA3 * tProfile->nTime + tProfile->dB3;
  }
  else
  {
    dPosTemp = tProfile->dPosMax;
  }

  // Control PID
  if(tMotor1.dir == tMotor2.dir)
  	{
	 g_nDutyCycle_1 = (int16_t)PIDCompute(tPIDControl1, g_dCmdVel + MPU6050.Gz, tMotor1.velocity, SAMPLING_TIME);
	  g_nDutyCycle_2 = (int16_t)(PIDCompute(tPIDControl2, g_dCmdVel - MPU6050.Gz, tMotor2.velocity, SAMPLING_TIME));
	}
  g_nDutyCycle_1 = (int16_t)PIDCompute(tPIDControl1, g_dCmdVel, tMotor1.velocity, SAMPLING_TIME);
  g_nDutyCycle_2 = (int16_t)(PIDCompute(tPIDControl2, g_dCmdVel, tMotor2.velocity, SAMPLING_TIME));

  dutyCycle_global_1 = g_nDutyCycle_1;
  dutyCycle_global_2 = g_nDutyCycle_2;


  MotorSetDuty(abs(g_nDutyCycle_1), MOTOR_1);
  MotorSetDuty(abs(g_nDutyCycle_2), MOTOR_2);

//  double real[7] = {tmotor1->velocity, tmotor2->velocity,  g_dCmdVel, tmotor1->position, tmotor2->position, dPosTemp, 0.0f};
//  strcpy(result, "=");
//  for (int i = 0; i < 7; i++) {
//        char buffer[25];
//        snprintf(buffer, sizeof(buffer), "%.2f", creal(real[i]));
//        strcat(result, buffer);
//
//        if (i < 6) {
//            strcat(result, ",");
//        }
//    }
//  strcat(result, "!");
//  HAL_UART_Transmit(&huart2, (uint8_t *)result, sizeof(result), 100);

  if (tProfile->nTime > tProfile->dMidStep3)
  {
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim3, 0);
    dPosTemp = 0;
    g_nDutyCycle_1 = 0;
    g_nDutyCycle_2 = 0;
    g_dCmdVel = 0;
    tProfile->nTime = 0;
    tProcess = NONE;
    MotorSetDuty(0, MOTOR_1);
    MotorSetDuty(0, MOTOR_2);
    tmotor1->velocity = 0;
    tmotor2->velocity = 0;
    tmotor1->position = 0;
    tmotor2->position = 0;
    tmotor1->counter = 0;
    tmotor2->counter = 0;
    PIDReset(&tPID_2);
    PIDReset(&tPID_1);
    if(tProfile->dMidStep3 < 2.0f)
    {
    for(uint64_t i = 0; i < 5000000; i++){};
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)statusOK, sizeof(statusOK), 1000);
    tProcess = NONE;
  }
  tProfile->nTime += SAMPLING_TIME;
}


/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
