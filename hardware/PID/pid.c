#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "car.h"
#include "usart.h"
#include <stdio.h>

float current_yaw = 0.0;
int left_pwm = 0;
int right_pwm = 0;

float target_angle = 75.0; // 假设目标方向为90度
float Kp = 1.0;
float Ki = 0;
float Kd = 0.01;
float integral = 0.0;
float previous_error = 0.0;
float error = 0;
float proportional, derivative, control;
int CarLeft90(void)
{
    current_yaw = get_yaw();
    printf("current_yaw=  %.2f\r\n", current_yaw);
    error = target_angle - current_yaw;
    printf("error=  %.2f\r\n", error);

    // 计算比例项、积分项、微分项
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - previous_error);

    // 计算控制量
    control = proportional + integral + derivative;
    printf("control=  %.2f\r\n", control);
    previous_error = error;
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
