#ifndef GET_SHOOT_INFO_H
#define GET_SHOOT_INFO_H
#include "main.h"
void Get_Shoot_Info(void const * argument);
void remote_ctrl_shoot_hook(void);
void send_shoot_motor_ctrl_message(int16_t shoot_cur[]);
#endif
