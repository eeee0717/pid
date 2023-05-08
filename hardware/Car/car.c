#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "car.h"

void car_stright(uint16_t pwm1,uint16_t pwm2) {
    // Set PWM duty cycle for TIM2 channel 1 and 3
	TIM2->CCR1=pwm1;
	TIM2->CCR2=pwm2;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|GPIO_PIN_6,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5|GPIO_PIN_7,GPIO_PIN_RESET); 	
 printf("car_stright");
 
	
}
void car_left(uint16_t pwm1,uint16_t pwm2) {
    // Set PWM duty cycle for TIM2 channel 1 and 3
    TIM2->CCR1=pwm1;
	TIM2->CCR2=pwm2;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|GPIO_PIN_6,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5|GPIO_PIN_7,GPIO_PIN_RESET); 	
	
}
void car_right(uint16_t pwm1,uint16_t pwm2) {
    // Set PWM duty cycle for TIM2 channel 1 and 3
  TIM2->CCR1=pwm1;
	TIM2->CCR2=pwm2;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|GPIO_PIN_6,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5|GPIO_PIN_7,GPIO_PIN_RESET); 	
	
}
