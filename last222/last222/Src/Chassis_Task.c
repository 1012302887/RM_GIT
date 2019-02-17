#include "main.h"
#include "arm_math.h"
#define FPU_COS arm_cos_f32
#define FPU_SIN arm_sin_f32
float line_angle;
/* chassis task global parameter */
ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;
chassis_t chassis = {0};//储存底盘处理各项信息的结构体
first_order_filter_type_t chassis_ref_first[4];
/* 底盘定时任务*/
int32_t ref1=0,ref2=0,ref3=0,ref4=0,fdb1=0,fdb2=0,fdb3=0,fdb4=0;
void Chassis_Task(void const *argument)
{
//	printf("--%f--",InfantryJudge.remainPower);
//	Ni_Ming(0xf1,gyro_data.yaw,gyro_data.yaw_angle,chassis.vw_offset,chassis.vw);
//	Ni_Ming(0xf2, chassis.wheel_spd_ref[0],chassis.wheel_spd_ref[1],chassis.wheel_spd_fdb[0],chassis.wheel_spd_fdb[1]);

	chassis.vw =  chassis_pid_calc (&pid_rotate,gyro_data.yaw,chassis.vw_offset);

	chassis.wheel_spd_ref[0] =  -chassis.vx_offset  +  chassis.vy_offset + chassis.vw;
	chassis.wheel_spd_ref[1] =   chassis.vx_offset  +  chassis.vy_offset + chassis.vw;
	chassis.wheel_spd_ref[2] =  -chassis.vx_offset  -  chassis.vy_offset + chassis.vw;
	chassis.wheel_spd_ref[3] =   chassis.vx_offset  -  chassis.vy_offset + chassis.vw;
	
	if(chassis.stop == 1)
	{
						for(int i =0; i < 4; i++)
		{
			chassis.wheel_spd_ref[i] = 0;
		}
	}
	
	for(int i =0; i < 4; i++)
	{
//		/*滤波*/
		chassis.wheel_spd_fdb[i] =	Kalman_filter_calc(&CHASSIS_KF[i],chassis.wheel_spd_fdb[i]);
//		/*滤波*/
		chassis.current[i] = chassis_pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);//
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
		PID_struct_init(&pid_rotate, POSITION_PID, 27, 0, 
	 1.9, 0, 0);//2.0
	
	 for (int k = 0; k < 4; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 3000, 0,
		300, 0, 0); 
	}
	for(int i =0;i<4;i++)
	{
		Kalman_filter_init(&CHASSIS_KF[i],1,1,40);//P-Q-R
	}
}
