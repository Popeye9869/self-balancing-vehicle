#include "pid.h"
#include "main.h"
#include "moto.h"
#include <stdint.h>

PID_TypeDef pid = {366.3, 8, 1.4652, 0.0f, 0.0f, 3000.0f};//{550, 3, 2.2}
PID_TypeDef pid_vel = {-0.3, -0.0015f, 0.3375, 0.0f, 0.0f, 15000.0f};//-0.22, -0.0, -0.08
PID_TypeDef pid_turn = {150, 0.0, 1.9, 0.0f, 0.0f, 3000.0f};

static float filteredVel = 0; // 低通滤波值

static float deadband = 0;//死区值
static float deadband_vel = 0;//速度死区值

float preVel = 0.7;

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    if(error < deadband_vel && error > -deadband_vel) error=0; //速度死区
    pid->integral += error;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = 0;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = 0;
    }
    float derivative = error - pid->pre_error;
    pid->pre_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

void PID_StandUpControl(float angle)
{
    int temp;

    float error = angle+roll_0 - roll;

    if(error < deadband && error > -deadband) error=0; //死区

    pid.integral += error;
    if (pid.integral > pid.integral_max)
    {
        vehicle_state = stop;//拿起检测
        pid.pre_error = 0;
        pid.integral = 0;

        pid_vel.integral = 0;
        pid_vel.pre_error = 0;
        preVel = 0;

        pid_turn.integral = 0;
        pid_turn.pre_error = 0;
    }
    else if (pid.integral < -pid.integral_max)
    {
        vehicle_state = stop;//拿起检测
        pid.pre_error = 0;
        pid.integral = 0;

        pid_vel.integral = 0;
        pid_vel.pre_error = 0;
        preVel = 0;

        pid_turn.integral = 0;
        pid_turn.pre_error = 0;
    }
    temp = (int)((pid.Kp * error) + (pid.Ki * pid.integral) - (pid.Kd * gx));

    MotoSetAdd(temp, temp);
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

void PID_TurnControl(float targetTurn)
{
    float temp;
    float error = targetTurn - yaw;
    if(error > 180) error -= 360;
    else if(error < -180) error += 360;
    if(error < deadband && error > -deadband) error=0; //死区
    pid_turn.integral += error;
    if (pid_turn.integral > pid_turn.integral_max)
    {
        pid_turn.integral = 0;
    }
    else if (pid_turn.integral < -pid_turn.integral_max)
    {
         pid_turn.integral = 0;
    }
    temp = (pid_turn.Kp * error) + (pid_turn.Ki * pid_turn.integral) - (pid_turn.Kd * gz);
    MotoSetAdd(-temp, temp);
}
