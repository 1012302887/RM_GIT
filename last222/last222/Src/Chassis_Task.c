#include "main.h"
#include "arm_math.h"
#define FPU_COS arm_cos_f32
#define FPU_SIN arm_sin_f32
/* chassis task global parameter */
ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;
chassis_t chassis = {0};//储存底盘处理各项信息的结构体
static float d_theta = 0;
extern osThreadId CAN_SEND_TASKHandle;
extern osThreadId GET_CHASSIS_INFHandle;
/* 底盘定时任务*/
void Chassis_Task(void const *argument)
{
//			Send_Pc_Data[1] = (uint8_t) gyro_data.yaw;
//			Send_Pc_Data[2] = (uint8_t) gyro_data.pitch;
//			printf("%f",gyro_data.yaw);
//	Ni_Ming(0xf1,gim.pid.pit_angle_fdb,gim.pid.yaw_angle_fdb,0,0);
//	USART6_Transmit();
	if(gim.ctrl_mode == GIMBAL_INIT)//chassis dose not follow gimbal when gimbal initializa
	{
		chassis.vw = 0;
	}
	else if(gim.ctrl_mode == GIMBAL_NORMAL)
	{
		chassis.vw = pid_calc(&pid_rotate, chassis.follow_gimbal, 0); //chassis.follow_gimbal = moto_yaw.total_angle
	}
	else if(gim.ctrl_mode == GIMBAL_WRITHE)
	{
		if((chassis.vx_offset == 0) && (chassis.vy_offset == 0))
		{
			if(chassis.follow_gimbal > STATIC_WRITHE_ANGLE_LIMIT)
			{
				chassis.writhe_speed_fac =  1;
			}
			else if(chassis.follow_gimbal < -STATIC_WRITHE_ANGLE_LIMIT)
			{
				chassis.writhe_speed_fac =  -1;
			}
		}
		else
		{
			if(chassis.follow_gimbal > RUN_WRITHE_ANGLE_LIMIT)
			{
				chassis.writhe_speed_fac =  1;
			}
			else if(chassis.follow_gimbal < -RUN_WRITHE_ANGLE_LIMIT)
			{
				chassis.writhe_speed_fac =  -1;
			}
		}	
		chassis.vw = pid_calc(&pid_rotate, chassis.writhe_speed_fac * WRITHE_SPEED_LIMIT, 0);
	}
	
//chassis.follow_gimbal = moto_yaw.total_angle;
/* run straight line with waist */
	if(chassis.follow_gimbal < 0){
		d_theta = chassis.follow_gimbal - 0;
	}
	else if(chassis.follow_gimbal <= 0+ 180 ){
		d_theta = chassis.follow_gimbal - 0;
	}
	else if(chassis.follow_gimbal <= 360){
		d_theta = 360 - chassis.follow_gimbal + 0;
	}
	d_theta /= -57.295;
	
	chassis.vx = chassis.vx_offset * FPU_COS(d_theta) - chassis.vy_offset * FPU_SIN(d_theta);
	chassis.vy = chassis.vx_offset * FPU_SIN(d_theta) + chassis.vy_offset * FPU_COS(d_theta);
	
	chassis.wheel_spd_ref[0] = -chassis.vx + chassis.vy + chassis.vw;
	chassis.wheel_spd_ref[1] =  chassis.vx + chassis.vy + chassis.vw;
	chassis.wheel_spd_ref[2] = -chassis.vx - chassis.vy + chassis.vw;
	chassis.wheel_spd_ref[3] =  chassis.vx - chassis.vy + chassis.vw;
	if(gim.stop == 1)
	{
			for(int i =0; i < 4; i++)
		{
			chassis.wheel_spd_ref[i] = 0;
		}
	}
	
	for(int i =0; i < 4; i++)
	{
		chassis.current[i] = chassis_pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
	}
	
	memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
	osSignalSet(CAN_SEND_TASKHandle, CHASSIS_MOTOR_MSG_SEND);
	osSignalSet(GET_CHASSIS_INFHandle, INFO_GET_SIGNAL);
}
/* 底盘参数初始化函数*/
void Chassis_Param_Init(void)
{
	memset(&chassis, 0, sizeof(chassis_t));
	
	chassis.writhe_speed_fac = -1;
	
	/* initializa chassis speed ramp */
	ramp_init(&LRSpeedRamp, MOUSR_LR_RAMP_TICK_COUNT);
	ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	
	/* initializa chassis follow gimbal pid */
		PID_struct_init(&pid_rotate, POSITION_PID, 40, 0, 
		1.7, 0, 0);//2.0
	
	 for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 8000, 0,
		400, 2, 0); 
	}
}
