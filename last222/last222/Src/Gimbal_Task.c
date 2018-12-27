#include "main.h"
#include "math.h"

/* gimbal task global parameter */
gimbal_t gim;
uint32_t gimbal_time_last;
int gimbal_time_ms;
uint32_t handler_run_time = 0;
ramp_t pit_ramp 	 = RAMP_GEN_DAFAULT;
ramp_t yaw_ramp    = RAMP_GEN_DAFAULT;

extern osThreadId GET_GIMBAL_INFOHandle;
extern osThreadId GET_CHASSIS_INFHandle;
extern osThreadId CAN_SEND_TASKHandle;

void Gimbal_Task(void const * argument)
{
	handler_run_time++;
//	gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
//	gimbal_time_last = HAL_GetTick();

	/* gimbal mode switch */
	 switch (gim.ctrl_mode)
	{
		case GIMBAL_INIT:
			init_mode_handler();
		break;
		
		case GIMBAL_NORMAL:
			normal_mode_handler();
		break;
		
		case GIMBAL_WRITHE:
			normal_mode_handler();
		break;
		
		default:
		break;
	}
	/* gimbal pit and yaw position pid */

//	gim.pid.pit_angle_ref = 0;
//	gim.pid.yaw_angle_ref = 0;
//	
	
	pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
	pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
	
	gim.pid.yaw_spd_ref = pid_yaw.out;
	gim.pid.pit_spd_ref = pid_pit.out;
 
	/* gimbal pid and yaw speed pid */
	gim.pid.yaw_spd_fdb = gim.sensor.yaw_palstance;
	gim.pid.pit_spd_fdb = gim.sensor.pit_palstance;
	
	pid_calc(&pid_yaw_spd, gim.pid.yaw_spd_fdb, gim.pid.yaw_spd_ref);
	pid_calc(&pid_pit_spd, gim.pid.pit_spd_fdb, gim.pid.pit_spd_ref);

	/* gimbal current out */
	glb_cur.gimbal_cur[0] = pid_yaw_spd.out;
	glb_cur.gimbal_cur[1] = pid_pit_spd.out;
//	glb_cur.gimbal_cur[2] = 0;
	glb_cur.gimbal_cur[2] = glb_cur.shoot_cur[2];
	
	osSignalSet(GET_GIMBAL_INFOHandle, GIMBAL_INFO_GET_SIGNAL);
	osSignalSet(CAN_SEND_TASKHandle, GIMBAL_MOTOR_MSG_SEND);
}

void init_mode_handler(void)
{
//	pid_yaw.p = 6;
	gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
	gim.pid.pit_angle_ref = 0;
	gim.pid.yaw_angle_ref = 0;
	if(handler_run_time > 2000)
	{
		gim.ctrl_mode = GIMBAL_NORMAL;
	}
}

void normal_mode_handler(void)
{
	gim.pid.pit_angle_fdb = gim.sensor.pit_relative_angle;
  gim.pid.yaw_angle_fdb = gim.sensor.yaw_relative_angle;
}

void Gimbal_Param_Init(void)
{
		for(int i =0;i<2;i++)
	{
		Kalman_filter_init(&zi_miao_kf[i],1,1,20);//P-Q-R
	}
	memset(&gim, 0, sizeof(gimbal_t));
	
	gim.ctrl_mode 		 = GIMBAL_INIT;
	gim.last_ctrl_mode = GIMBAL_INIT;
	
	/* pitch and yaw ramp Initializa */
	ramp_init(&pit_ramp, PIT_PREPARE_TIMS_MS);
	ramp_init(&yaw_ramp, YAW_PREPARE_TIMS_MS);
	/* pitch axis motor pid parameter */
  PID_struct_init(&pid_pit, POSITION_PID, 1000, 1000,
                  8, 0, 0.003); //30
  PID_struct_init(&pid_pit_spd, POSITION_PID, 6000, 2000,
                 29, 0, 0.003); //60

  /* yaw axis motor pid parameter */
  PID_struct_init(&pid_yaw, POSITION_PID, 1000, 1000,
                  11, 0, 0); //
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 6000, 2000,
                  29.5, 0.05, 0);
	Send_Pc_Data[0] = 0xAA;
	Send_Pc_Data[3] = 0xBB;
}
