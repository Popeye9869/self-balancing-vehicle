#ifndef __MOTO_H_
#define __MOTO_H_

#include "main.h"
#include "TIM.h"

void MotoControl(int left, int right);

int ReadVel(TIM_HandleTypeDef *htim);

#endif
