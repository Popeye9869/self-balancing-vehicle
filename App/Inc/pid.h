#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float pre_error;
    float integral;
} PID_TypeDef;


float PID_Compute(PID_TypeDef *pid, float setpoint, float measured_value);

void PID_StandUpControl(float angle);

#endif