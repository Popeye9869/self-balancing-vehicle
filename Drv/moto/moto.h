#ifndef __MOTO_H_
#define __MOTO_H_

#include "main.h"
#include "TIM.h"

void MotoControl(int left, int right);

void MotoSetAdd(int left_add, int right_add);

void MotoRun();

void MotoStop();

int ReadVel(TIM_HandleTypeDef *htim);

#endif
