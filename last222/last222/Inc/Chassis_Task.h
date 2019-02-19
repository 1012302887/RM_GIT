#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"

#define RUN_WRITHE_ANGLE_LIMIT					  25
#define STATIC_WRITHE_ANGLE_LIMIT					50
#define WRITHE_SPEED_LIMIT					      30.0f/1.3f

/* chassis speed ramp */
#define MOUSR_LR_RAMP_TICK_COUNT		200
#define MOUSR_FB_RAMP_TICK_COUNT		100


typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
	float           vy_foreword_leg; 
	float           vy_back_leg; 
  float           vw; // rotate
  float        		vx_offset;
  float        		vy_offset; 
	float           vw_offset;
	
	float         	follow_gimbal;
	float					writhe_speed_fac;
  
//  chassis_mode_e  ctrl_mode;
//  chassis_mode_e  last_ctrl_mode;

	float           wheel_pos_fdb[4];
  float           wheel_pos_ref[4];
	
  float           wheel_spd_fdb[4];
  float           wheel_spd_ref[4];
	
  int16_t         current[4];
	float         	power_surplus;
  uint8_t   stop;
	float vleg;
} chassis_t;

void Chassis_Task(void const * argument);
void Chassis_Param_Init(void);
extern chassis_t chassis;
#endif
