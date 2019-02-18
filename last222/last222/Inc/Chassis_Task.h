#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"

#define RUN_WRITHE_ANGLE_LIMIT					  15
#define STATIC_WRITHE_ANGLE_LIMIT					20
#define WRITHE_SPEED_LIMIT					      30/1.3f

/* chassis speed ramp */
#define MOUSR_LR_RAMP_TICK_COUNT		500
#define MOUSR_FB_RAMP_TICK_COUNT		500


typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // rotate
  float        		vx_offset;
  float        		vy_offset; 
	float           vw_offset;
	
	int16_t        	follow_gimbal;
	int16_t					writhe_speed_fac;
  
//  chassis_mode_e  ctrl_mode;
//  chassis_mode_e  last_ctrl_mode;

	float           wheel_pos_fdb[4];
  float           wheel_pos_ref[4];
	
  float           wheel_spd_fdb[4];
  float           wheel_spd_ref[4];
	
  int16_t         current[4];
	float         	power_surplus;
  
} chassis_t;

void Chassis_Task(void const * argument);
void Chassis_Param_Init(void);
extern chassis_t chassis;
#endif
