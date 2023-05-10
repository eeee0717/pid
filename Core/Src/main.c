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
#include "mpu6050.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "car.h"
#include "servo.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t uart1_buffer[1]; // ����һ������Ϊ10�����飬���ڴ洢���յ�������
uint8_t uart1_flag = 0;  // uart1_flag
uint8_t uart2_buffer[1]; // ����һ������Ϊ10�����飬���ڴ洢���յ�������
uint8_t uart2_flag = 0;
uint8_t uart3_buffer[1]; // ����һ������Ϊ10�����飬���ڴ洢���յ�������
uint8_t uart3_flag = 0;  // uart1_flag
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ��������
void Mpu6050_Init(void);
void Problem1(void);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // float current_yaw = 0.0;
  // int left_pwm = 0;
  // int right_pwm = 0;

  // float target_angle = 75.0; // ����Ŀ�귽��Ϊ90��
  // float Kp = 1.0;
  // float Ki = 0;
  // float Kd = 0.01;
  // float integral = 0.0;
  // float previous_error = 0.0;
  // float error = 0;
  // float proportional, derivative, control;
  int car_left_uart1_flag = 0;
  int problem1_flag = 0;

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&huart1, uart1_buffer, sizeof(uart1_buffer));
  HAL_UART_Receive_IT(&huart2, uart2_buffer, sizeof(uart2_buffer));
  HAL_UART_Receive_IT(&huart3, uart3_buffer, sizeof(uart3_buffer));

  // �����ʼ��
  // servo_init();
  Mpu6050_Init();
  // ��ת
  // motor_turn_left();
  // ��ת
  motor_forward();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    Problem1();

    // if (problem1_flag == 0)
    // {
    //   problem1_flag = CarStraight();
    // }

    // if (problem1_flag == 1)
    // {
    //   motor_turn_left();
    // CarLeft90();
    // }
    // UsartTest();
    // ���Դ�����
    // if (uart2_flag == 1)
    // {
    //   if (uart2_buffer[0] == 'a')
    //   {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    //     delay_ms(100);
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    //     delay_ms(100);
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    //     delay_ms(100);
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    //     uart2_flag = 0;
    //   }
    // }

    // car_stright(left_pwm, right_pwm);
    // delay_ms(100);
    // printf("yaw=  %.2f\r\n", current_yaw);
    // car_stright(3500, 3500);
    // if (uart1_flag == 1)
    // {
    //   if (uart1_buffer[0] == 'a')
    //   {
    //     printf("uart1_buffer: %d", uart1_buffer[0]);
    //     car_stright(3500, 3500);
    //   }
    //   if (uart1_buffer[0] == 'b')
    //   {
    //     car_left(3500, 1500);
    //   }
    //   if (uart1_buffer[0] == 'c')
    //   {
    //     car_right(1500, 3500);
    //   }
    //   if (uart1_buffer[0] == 'd')
    //   {
    //     car_right(0, 0);
    //   }
    //   uart1_flag = 0;
    // }
    //		pid_control(0,get_angle());
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {

    // ���յ����ݺ󣬽���洢��buffer������
    HAL_UART_Receive_IT(&huart1, uart1_buffer, sizeof(uart1_buffer)); // �ٴ����ô��ڽ����жϣ��Խ�����һ������
    uart1_flag = 1;

    // HAL_UART_Transmit(&huart1, uart1_buffer, sizeof(uart1_buffer), HAL_MAX_DELAY);
  }
  if (huart->Instance == USART2)
  {

    // ���յ����ݺ󣬽���洢��buffer������
    HAL_UART_Receive_IT(&huart2, uart2_buffer, sizeof(uart2_buffer)); // �ٴ����ô��ڽ����жϣ��Խ�����һ������
    uart2_flag = 1;
    // ���ܷ����������˻�����
    // HAL_UART_Transmit(&huart1, uart2_buffer, sizeof(uart2_buffer), HAL_MAX_DELAY);
  }
}

void Mpu6050_Init(void)
{
  while (MPU_Init())
    ; // ��ʼ��MPU6050
  printf("%s\r\n", "MPU Init...");
  while (mpu_dmp_init())
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    delay_ms(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
    printf("%s\r\n", "Mpu6050 Init Wrong!");
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
  printf("%s\r\n", "Mpu6050 Init OK!");
}

void Problem1(void)
{
  CarStraight();
  //  UsartTest();

  if (uart1_flag == 1)
  {
    if (uart1_buffer[0] == 'b')
    {
      while (1)
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
        delay_ms(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
        delay_ms(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
        delay_ms(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0��Ӧ����PB5���ͣ�������ͬ��LED0(0)
        car_stop();
      }
      // flag = camera_confirm(2);
      // while (flag)
      // {
      //   UsartTest();
      // }
    }
    uart1_buffer[0] = 0;
    uart1_flag = 0;
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
