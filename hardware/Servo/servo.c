#include "servo.h"
#include "tim.h"

void servo_init(void)
{
    TIM3->CCR3 = 45;
    TIM3->CCR4 = 200;
}
