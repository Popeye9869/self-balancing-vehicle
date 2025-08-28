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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "inv_mpu.h"
#include "oled.h"
#include "mpu6050.h"
#include "moto.h"
#include "pid.h"
#include <stdio.h>
#include <sys/_intsup.h>
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

enum my_state vehicle_state = unintialized;

int flag = 0;

float pitch, roll, yaw;

float roll_0 = -4.8; //初始角度

short gx, gy, gz; //陀螺仪原始数据
short ax, ay, az; //加速度原始数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //启动编码器接口
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //启动编码器接口

  
  //初始化MPU6050
  if(MPU_Init())
  {
    OLED_NewFrame();
    OLED_PrintString(0, 0, "MPU6050 Error!", &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    while (1)
      ;
  }
  else
  {
    OLED_NewFrame();
    OLED_PrintString(0, 0, "MPU6050 OK!", &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }

  //初始化DMP
  if (mpu_dmp_init())
  {
    OLED_NewFrame();
    OLED_PrintString(0, 0, "DMP Error!", &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    while (1)
      ;
  }
  else
  {
    OLED_NewFrame();
    OLED_PrintString(0, 0, "DMP OK!", &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  vehicle_state = running;

  HAL_TIM_Base_Start_IT(&htim3); //启动定时器3中断

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle the LED on PC13
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MPU_Get_Gyroscope(&gx,  &gy, &gz);
    MPU_Get_Accelerometer(&ax, &ay, &az);
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    // OLED_NewFrame();
    // char buf[20];
    // snprintf(buf, sizeof(buf), "P:%.2f", pitch);
    // OLED_PrintString(0, 0, buf, &font16x16, OLED_COLOR_NORMAL);
    // snprintf(buf, sizeof(buf), "R:%.2f", roll);
    // OLED_PrintString(0, 16, buf, &font16x16, OLED_COLOR_NORMAL);
    // snprintf(buf, sizeof(buf), "Y:%.2f", yaw);
    // OLED_PrintString(0, 32, buf, &font16x16, OLED_COLOR_NORMAL);
    // OLED_ShowFrame();


    // OLED_NewFrame();
    // char buf[20];
    // sprintf(buf, "tim2:%d", ReadVel(&htim2));
    // OLED_PrintString(0, 0, buf, &font16x16, OLED_COLOR_NORMAL);
    // sprintf(buf, "tim4:%d", ReadVel(&htim4));
    // OLED_PrintString(0, 16, buf, &font16x16, OLED_COLOR_NORMAL);
    // OLED_ShowFrame();
    // HAL_Delay(1000);

    // OLED_NewFrame();
    // char buf[20];
    // sprintf(buf, "ax:%d", ax);
    // OLED_PrintString(0, 0, buf, &font16x16, OLED_COLOR_NORMAL);
    // sprintf(buf, "ay:%d", ay);
    // OLED_PrintString(0, 16, buf, &font16x16, OLED_COLOR_NORMAL);
    // sprintf(buf, "az:%d", az);
    // OLED_PrintString(0, 32, buf, &font16x16, OLED_COLOR_NORMAL);
    // OLED_ShowFrame();

    // OLED_NewFrame();
    // char buf[20];
    // sprintf(buf, "gx:%d", gx);
    // OLED_PrintString(0, 0, buf, &font16x16, OLED_COLOR_NORMAL);
    // sprintf(buf, "gy:%d", gy);
    // OLED_PrintString(0, 16, buf, &font16x16, OLED_COLOR_NORMAL);
    // sprintf(buf, "gz:%d", gz);
    // OLED_PrintString(0, 32, buf, &font16x16, OLED_COLOR_NORMAL);
    // OLED_ShowFrame();

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

/* USER CODE BEGIN 4 */
__used void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && vehicle_state == running) // Check if the interrupt is from TIM2
    {
        // Call your PID control function here
        //PID_StandUpControl(0);
        PID_VelControl(0);
    }
    if (htim->Instance == TIM3 && vehicle_state == stop) // Check if the interrupt is from TIM2
    {
      if((roll - roll_0) > -1 && (roll - roll_0) < 1)
        flag++;
      if (flag > 50) {
        vehicle_state = running;
        flag = 0;
      
      }
    }
    else {
      MotoControl(0, 0);
    }
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
