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
#include "math.h"
#include "car.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t uart1_buffer[1]; // 定义一个长度为10的数组，用于存储接收到的数据
uint8_t flag=0; // flag
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
// 定义PID参数
float kp = 1.0; // 比例系数
float ki = 0.5; // 积分系数
float kd = 0.1; // 微分系数
float dt = 0.01; // 时间间隔

// 定义PID变量
float pid_error = 0;
float pid_last_error = 0;
float pid_integral = 0;
float pid_derivative = 0;
float pid_output = 0;
float get_angle(void)
{
  // 读取加速度计和陀螺仪数据
//  uint8_t buffer[14];
//  MPU_Read_Len( MPU6050_ADDR,MPU6050_ACCEL_XOUT_H,14,buffer);

//  int16_t ax = (buffer[0] << 8) | buffer[1];
//  int16_t ay = (buffer[2] << 8) | buffer[3];
//  int16_t az = (buffer[4] << 8) | buffer[5];
//  int16_t gx = (buffer[8] << 8) | buffer[9];
//  int16_t gy = (buffer[10] << 8) | buffer[11];
//  int16_t gz = (buffer[12] << 8) | buffer[13];
  
  // 计算角度
//  float angle_x = atan2f((float)ax, sqrtf(powf((float)ay, 2) + powf((float)az, 2))) * 180 / 3.1415926;
//  float angle_y = atan2f((float)ay, sqrtf(powf((float)ax, 2) + powf((float)az, 2))) * 180 / 3.1415926;
//  float angle_z = atan2f(sqrtf(powf((float)ax, 2) + powf((float)ay, 2)), (float)az) * 180 / 3.1415926;
	float pitch,roll,yaw; 		//欧拉角
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){};
	printf("pitch=  %.2f\t",pitch);
	printf("roll=  %.2f\t",roll);
	printf("yaw=  %.2f\r\n",yaw);
	
	
  return yaw;
	
}
void pid_control(float target_angle, float current_angle)
{
  // 计算误差
  pid_error = target_angle - current_angle;
  
  // 计算积分项
  pid_integral += pid_error * dt;
  
  // 计算微分项
  pid_derivative = (pid_error - pid_last_error) / dt;
  pid_last_error = pid_error;
  
  // 计算PID输出
  pid_output = kp * pid_error + ki * pid_integral + kd * pid_derivative;
  
  // 输出PID输出
//  printf("PID output: %.2f\r\n", pid_output);
}

// 函数声明
void Mpu6050_Init(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		float  current_yaw;
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
  
  /* USER CODE BEGIN 2 */
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	HAL_TIM_Base_Start(&htim2); 
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_UART_Receive_IT(&huart1, uart1_buffer, sizeof(uart1_buffer));
	

	Mpu6050_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		current_yaw=get_angle();

		if(flag==1){
		 if(uart1_buffer[0]=='a'){
			 printf("uart1_buffer: %d",uart1_buffer[0]);
			 car_stright(3500,3500);
		 }
		 if(uart1_buffer[0]=='b'){
			 car_left(3500,1500);
		 }
		 if(uart1_buffer[0]=='c'){
			 car_right(1500,3500); 
		 }
		 if(uart1_buffer[0]=='d'){
			 car_right(0,0); 
		 }
		 flag=0;
		 }
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
		
    // 接收到数据后，将其存储到buffer数组中
    HAL_UART_Receive_IT(&huart1, uart1_buffer, sizeof(uart1_buffer)); // 再次启用串口接收中断，以接收下一批数据
		flag=1;
		
//		HAL_UART_Transmit(&huart1, uart1_buffer, sizeof(uart1_buffer), HAL_MAX_DELAY);
		
  }

}

void Mpu6050_Init(void)
{
	while(MPU_Init());					//初始化MPU6050
	printf("%s\r\n","MPU Init...");
	while(mpu_dmp_init())
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET); 	//LED0对应引脚PB5拉低，亮，等同于LED0(0)
		delay_ms(100);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET); 	//LED0对应引脚PB5拉低，亮，等同于LED0(0)
		printf("%s\r\n","Mpu6050 Init Wrong!");

	}
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET); 	//LED0对应引脚PB5拉低，亮，等同于LED0(0)
	printf("%s\r\n","Mpu6050 Init OK!");
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
