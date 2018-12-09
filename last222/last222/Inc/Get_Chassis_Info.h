#ifndef Get_Chassis_Info_H
#define Get_Chassis_Info_H
#include "main.h"
void Get_Chassis_Info(void const * argument);
void remote_ctrl_chassis_hook(void);
void send_chassis_motor_ctrl_message(int16_t chassis_cur[]);
/* chassis remote control velocity factor */
#define CHASSIS_REF_FACT   			60.0/660.0
#define CHASSIS_ROTATE_FACT			3.0f

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


