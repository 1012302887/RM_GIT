#ifndef Get_Chassis_Info_H
#define Get_Chassis_Info_H
#include "main.h"
void Get_Chassis_Info(void const * argument);
void remote_ctrl_chassis_hook(void);
void send_chassis_motor_ctrl_message(int16_t chassis_cur[]);
void keyboard_chassis_hook(void);
/* chassis remote control velocity factor */
#define CHASSIS_REF_FACT   			60.0/660.0
#define CHASSIS_ROTATE_FACT			3.0f
#define NORMAL_FORWARD_BACK_SPEED 			40.0f  						 //normal forward   speed
#define NORMAL_LEFT_RIGHT_SPEED   			30.0f							 //normal rightward speed
#define HIGH_FORWARD_BACK_SPEED 			  60.0f							 //faster forward   speed								
#define HIGH_LEFT_RIGHT_SPEED   				40.0f							 //faster rightward speed
#define MOUSE_TO_PITCH_ANGLE_INC_FACT   0.01f
#define MOUSE_TO_YAW_ANGLE_INC_FACT     0.01f 
/* limit handler */
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}

#endif


