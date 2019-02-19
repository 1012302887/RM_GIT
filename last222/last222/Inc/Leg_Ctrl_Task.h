#ifndef LEG_CTRL_TASK_H_
#define LEG_CTRL_TASK_H_
#include "main.h"
extern chassis_t chassis_leg ;//储存底盘上面四腿电机处理各项信息的结构体
void Leg_Param_Init(void);
void Leg_Ctrl_Task(void const *argument);
void Leg_task_init(void);
void Leg_task_normal(void);
extern uint8_t leg_mode;
extern uint32_t handle_run_time;
extern ramp_t LegSpeedRamp ;
typedef enum
{
	leg_init_mode = 0,
	leg_normal_mode =1,
} leg_crtl_mode_e;
#endif
