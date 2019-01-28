#ifndef __COMMON_TASK_H__
#define __COMMON_TASK_H__

#include "main.h"

#define LEG_INFO_GET_SIGNAL ( 1 << 3 )
#define LEG_MOTOR_MSG_SEND   ( 1 << 4 )
#define GIMBAL_MOTOR_MSG_GET   ( 1 << 5 )
#define GIMBAL_MOTOR_MSG_SEND  ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND ( 1 << 7 )
#define SHOOT_GET_SIGNAL       ( 1 << 8 )
#define INFO_GET_SIGNAL   		 ( 1 << 9 )

typedef struct
{
  /* 4 chassis motor current */
  int16_t chassis_cur[4];
  /* yaw/pitch motor current */
  int16_t gimbal_cur[3];
	/* friction/trigger motor current */
	int16_t shoot_cur[3];
} motor_current_t;

void Can_Send_Task(void const *argument);

extern motor_current_t glb_cur;

#endif
