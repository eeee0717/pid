#include "car.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// 电机正转
void motor_forward(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);
}
// 左转
void motor_turn_left(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_RESET);
}
// 右转
void motor_turn_right(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
}
// 后退
void motor_backward(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_RESET);
}

void car_stright(uint16_t pwm1, uint16_t pwm2)
{
  // Set PWM duty cycle for TIM2 channel 1 and 3
  // __HAL_TIM_SetCompare()
  TIM2->CCR1 = pwm1;
  TIM2->CCR2 = pwm2;
}
void car_left(uint16_t pwm1, uint16_t pwm2)
{
  // Set PWM duty cycle for TIM2 channel 1 and 3
  TIM2->CCR1 = pwm1;
  TIM2->CCR2 = pwm2;
}
void car_right(uint16_t pwm1, uint16_t pwm2)
{
  // Set PWM duty cycle for TIM2 channel 1 and 3
  TIM2->CCR1 = pwm1;
  TIM2->CCR2 = pwm2;
}
void car_stop(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;
}
