#include "pid.h"
#include "moto.h"

PID_TypeDef pid = {100, 0, 0, 0.0f, 0.0f};

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    pid->integral += error;
    float derivative = error - pid->pre_error;
    pid->pre_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

void PID_StandUpControl(float angle)
{
    int temp;
    temp = (int)PID_Compute(&pid, roll_0, roll);
    MotoControl(temp, temp);
}
