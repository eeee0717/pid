#include "servo.h"
#include "tim.h"

void servo_init(void)
{
    TIM3->CCR3 = 190;
    TIM3->CCR4 = 200;
}
