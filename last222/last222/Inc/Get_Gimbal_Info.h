#ifndef Get_Gimbal_Info_H
#define Get_Gimbal_Info_H
#include "main.h"
void Get_Gimbal_Info(void const * argument);
void get_gimbal_info(void);
void remote_ctrl_gimbal_hook(void);
void keyboard_gimbal_hook(void);
void GimbalAngleLimit(void);
void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[]);
/* gimbal remote control angle factor */
#define GIMBAL_PIT_REF_FACT 		0.0005f
#define GIMBAL_YAW_REF_FACT     0.001f
#define MOUSE_TO_PITCH_ANGLE_INC_FACT   0.01f
#define MOUSE_TO_YAW_ANGLE_INC_FACT     0.01f 

#endif
