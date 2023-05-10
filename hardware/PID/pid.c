#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "car.h"
#include <stdio.h>
#include "main.h"

/** 示例代码
    car_left_flag = CarLeft90();
    if (car_left_flag == 1)
    break;
*/
float left_previous_error = 0.0;
float straight_previous_error = 0.0;

void UsartTest(void)
{

    if (uart2_flag == 1)
    {
        if (uart2_buffer[0] == 'a')
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0对应引脚PB5拉低，亮，等同于LED0(0)
            delay_ms(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0对应引脚PB5拉低，亮，等同于LED0(0)
            delay_ms(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0对应引脚PB5拉低，亮，等同于LED0(0)
            delay_ms(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED0对应引脚PB5拉低，亮，等同于LED0(0)
        }
        uart2_flag = 0;
    }
}
int CarLeft90(void)
{

    float current_yaw = 0.0;
    int left_pwm = 0;
    int right_pwm = 0;

    float target_angle = 78.0; // 假设目标方向为90度
    float Kp = 1.0;
    float Ki = 0;
    float Kd = 0.01;
    float integral = 0.0;
    float error = 0;
    float proportional, derivative, control;
    motor_turn_left();
    current_yaw = get_yaw();
    printf("current_yaw=  %.2f\r\n", current_yaw);
    error = target_angle - current_yaw;
    printf("error=  %.2f\r\n", error);

    // 计算比例项、积分项、微分项
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - left_previous_error);

    // 计算控制量
    control = proportional + integral + derivative;
    printf("control=  %.2f\r\n", control);
    control = control / 3;
    left_previous_error = error;
    left_pwm = 500 + control;
    right_pwm = 500 + control;
    car_left(left_pwm, right_pwm);
    if (fabs(error) < 1.5)
    {
        // 停止控制循环，小车已到达目标方向
        return 1;
    }
    return 0;
}

int CarRight90(void)
{

    float current_yaw = 0.0;
    int left_pwm = 0;
    int right_pwm = 0;

    float target_angle = 12; // 假设目标方向为90度
    float Kp = 1.0;
    float Ki = 0;
    float Kd = 0.01;
    float integral = 0.0;
    float error = 0;
    float proportional, derivative, control;

    motor_turn_right();
    current_yaw = get_yaw();
    printf("current_yaw=  %.2f\r\n", current_yaw);
    error = target_angle - current_yaw;
    printf("error=  %.2f\r\n", error);

    // 计算比例项、积分项、微分项
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - left_previous_error);

    // 计算控制量
    control = proportional + integral + derivative;
    printf("control=  %.2f\r\n", control);
    left_previous_error = error;
    control = control / 3;
    left_pwm = 500 - control;
    right_pwm = 500 - control;
    car_right(left_pwm, right_pwm);
    if (fabs(error) < 1.5)
    {
        // 停止控制循环，小车已到达目标方向
        return 1;
    }
    return 0;
}

void CarStraight(void)
{

    float current_yaw = 0.0;
    int left_pwm = 500;
    int right_pwm = 500;

    float target_angle = 0.0;
    float Kp = 1.0;
    float Ki = 0;
    float Kd = 0.01;
    float integral = 0.0;
    float error = 0;
    float proportional, derivative, control;

    current_yaw = get_yaw();
    printf("current_yaw=  %.2f\r\n", current_yaw);
    error = target_angle - current_yaw;
    printf("error=  %.2f\r\n", error);

    // 计算比例项、积分项、微分项
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - straight_previous_error);

    // 计算控制量
    control = proportional + integral + derivative;
    control = control / 3;
    // printf("control=  %.2f\r\n", control);
    straight_previous_error = error;
    // 左偏error < 0
    if (error < 0)
    {
        left_pwm = 550 + control;
        right_pwm = 500 - control;
    }
    else if (error > 0)
    {
        left_pwm = 500 - control;
        right_pwm = 550 + control;
    }
    if (left_pwm < 450)
        left_pwm = 450;
    if (right_pwm < 450)
        right_pwm = 450;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED0对应引脚PB5拉低，亮，等同于LED0(0)

    car_stright(left_pwm, right_pwm);
    // return flag_camera;
}
void CarBack(void)
{
    car_stright(550, 600);
}
