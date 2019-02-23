#include "main.h"
uint32_t handle_run_time;
uint8_t leg_mode = leg_init_mode;
uint8_t flag ,flag1=0;uint32_t leg_ctrl_time=0;
chassis_t chassis_leg = {0};//储存底盘上面四腿电机处理各项信息的结构体
ramp_t LegSpeedRamp = RAMP_GEN_DAFAULT;
void Leg_Ctrl_Task(void const *argument)
{
	handle_run_time++;
	if(leg_mode == leg_init_mode)
		{
			leg_ctrl_time++;
			Leg_task_init();
		}
	else
		{
			Leg_task_normal();
		}
	for(int i =0; i < 4; i++)
	{
		chassis_leg.current[i]=chassis_pid_calc(&pid_leg_spd[i], chassis_leg.wheel_spd_fdb[i], chassis_leg.wheel_spd_ref[i]);//
	} 
		osSignalSet(CAN_SEND_TASKHandle, LEG_MOTOR_MSG_SEND);
		osSignalSet(GET_LEG_INFHandle, LEG_INFO_GET_SIGNAL);
	
}
void Leg_task_normal(void)
{
		chassis_leg.wheel_spd_ref[0] =  chassis_leg.vx +chassis_leg.vy_foreword_leg ;
		chassis_leg.wheel_spd_ref[1] =  chassis_leg.vx +chassis_leg.vy_foreword_leg ;
		chassis_leg.wheel_spd_ref[2] =  chassis_leg.vx +chassis_leg.vy_back_leg ;
		chassis_leg.wheel_spd_ref[3] =  chassis_leg.vx +chassis_leg.vy_back_leg ;
}
void Leg_task_init(void)
{
	if(leg_ctrl_time<=2000)
	{
		chassis_leg.wheel_spd_ref[0] =  -18*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[1] =  -18*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[2] =  -18*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[3] =  -18*ramp_calc(&LegSpeedRamp);
	}
	else
	{
		flag=1;
	}
	if(flag1==0&&flag==1)
	{
		for(int i =0; i < 4; i++)
		{
			memset(&moto_chassis[i], 0, sizeof(moto_chassis[i]));
		}
		flag1=1;
	}
	if(flag!=0)
	{
		chassis_leg.wheel_spd_ref[0] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[1] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[2] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[3] =  28*ramp_calc(&LegSpeedRamp);
	if(moto_chassis[0].total_angle>=34500||moto_chassis[1].total_angle>=34500||moto_chassis[2].total_angle>=34500||\
			moto_chassis[3].total_angle>=34500)
		{
			chassis_leg.wheel_spd_ref[0] =  0;
			chassis_leg.wheel_spd_ref[1] =  0;
			chassis_leg.wheel_spd_ref[2] =  0;
			chassis_leg.wheel_spd_ref[3] =  0;
			leg_mode = leg_normal_mode;
			ramp_init(&LegSpeedRamp,1500);
		}
	}
}
/* 四腿参数初始化函数*/
void Leg_Param_Init(void)
{
	leg_mode = leg_init_mode;
	ramp_init(&LegSpeedRamp,1000);
	memset(&chassis_leg, 0, sizeof(chassis_t));
	 for (int k = 0; k < 4; k++)
  {
		PID_struct_init(&pid_leg_spd[k], POSITION_PID, 10000, 0,
		350, 0, 0); 
	}
}
