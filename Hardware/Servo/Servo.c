#include <stm32f10x.h>
#include "Servo.h"
#include "PWM.h"
void Servo_Init()
{
    PMW_Init();
}

//0.5ms <-20ms 0度
//2.5ms <-20ms 180度
void Servo_SetAngle(double Angle)
{
    PWM_SetDuty(Angle/180*10+2.5);
}