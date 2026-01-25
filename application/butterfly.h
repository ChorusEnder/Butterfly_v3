# pragma once

#include "main.h"

typedef enum {
    BUTTERFLY_MODE_STOP,//急停模式

    BUTTERFLY_MODE_POSITION,// 定点模式:通过遥控器控制翅膀位置
    BUTTERFLY_MODE_FLYING,// 飞行模式:通过遥控器控制翅膀速度,转向,升降...

    BUTTERFLY_MODE_MECHANISM,// 机构模式:通过遥控器控制振频
} butterfly_mode_e;


void Butterfly_Init();
void Butterfly_Task();

