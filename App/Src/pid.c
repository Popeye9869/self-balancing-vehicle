#include "pid.h"
#include "main.h"
#include "moto.h"
#include <stdint.h>

PID_TypeDef pid = {366.3, 8, 1.4652, 0.0f, 0.0f, 3000.0f};//{550, 3, 2.2}
PID_TypeDef pid_vel = {-0.22, -0., -0., 0.0f, 0.0f, 15000.0f};

static float filteredVel = 0.3f; // 低通滤波值

static float deadband = 0.5f;//死区值

float preVel = 0;

float my_abs(float a) {
    return a > 0 ? a : -a;
}

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    if(my_abs(error)<deadband) error=0; //死区
    pid->integral += error;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = 0;
        vehicle_state = stop;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = 0;
        vehicle_state = stop;
    }
    float derivative = error - pid->pre_error;
    pid->pre_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

void PID_StandUpControl(float angle)
{
    int temp;

    float error = angle+roll_0 - roll;
    pid.integral += error;
    if (pid.integral > pid.integral_max)
        pid.integral = 0;
    else if (pid.integral < -pid.integral_max)
        pid.integral = 0;
    temp = (int)((pid.Kp * error) + (pid.Ki * pid.integral) - (pid.Kd * gx));

    //temp = (int)PID_Compute(&pid, angle+roll_0, roll);
    MotoControl(temp, temp);
}

void PID_VelControl(int16_t targetVel)
{
    float temp;
    // 读取当前速度
    int16_t currentVel = ReadVel(&htim2)-ReadVel(&htim4);//左减右
    //低通滤波
    float vel = (filteredVel * preVel + (1 - filteredVel) * currentVel);
    preVel = vel;
    // 计算PID输出
    temp = PID_Compute(&pid_vel, targetVel, vel);
    PID_StandUpControl(temp);
}
