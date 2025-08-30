#include "moto.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx.h"

int left_vel = 0;
int right_vel = 0; 


int abs(int p)
{
    return p > 0 ? p : -p;
}
void MotoControl(int left, int right)
{
    if (left > 7200)
        left = 7200;
    if (left < -7200)
        left = -7200;
    if (right > 7200)
        right = 7200;
    if (right < -7200)
        right = -7200;
    if (left > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(left));
    }
    else if (left < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(left));
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    if (right > 0)
    {
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(right));
    }
    else if (right < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(right));
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

int ReadVel(TIM_HandleTypeDef *htim)
{
    int val = __HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return val;
}

void MotoSetAdd(int left_add, int right_add)
{
    left_vel += left_add;
    right_vel += right_add;
    if (left_vel > 7200)
        left_vel = 7200;
    if (left_vel < -7200)
        left_vel = -7200;
    if (right_vel > 7200)
        right_vel = 7200;
    if (right_vel < -7200)
        right_vel = -7200;
}

void MotoRun()
{
    MotoControl(left_vel, right_vel);
    left_vel = 0;
    right_vel = 0;
}

void MotoStop()
{
    left_vel = 0;
    right_vel = 0;
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    MotoControl(0, 0);
}
