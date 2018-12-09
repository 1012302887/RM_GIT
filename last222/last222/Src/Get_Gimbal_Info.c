#include "main.h"
ramp_t pit_ramp 	 = RAMP_GEN_DAFAULT;
ramp_t yaw_ramp    = RAMP_GEN_DAFAULT;
/* USER CODE END Header_Get_Gimbal_Info */
void Get_Gimbal_Info(void const * argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(GIMBAL_INFO_GET_SIGNAL,osWaitForever);
		if(event.value.signals & GIMBAL_INFO_GET_SIGNAL)
		{   
			  taskENTER_CRITICAL();
				get_gimbal_info();
			  taskEXIT_CRITICAL();
				//Ni_Ming(0xf1,moto_friction[0].ecd,moto_friction[1].ecd,moto_trigger.ecd,0);
			  //Ni_Ming(0xf2,gyro_data.pitch,gyro_data.yaw,0,0);
		}
	}
}
void get_gimbal_info(void)
{
		if(gim.ctrl_mode == GIMBAL_INIT)
	{
		/* get gimbal relative angle */
		gim.sensor.yaw_relative_angle = moto_yaw.total_angle * ramp_calc(&yaw_ramp);
		gim.sensor.pit_relative_angle = gyro_data.pitch * ramp_calc(&pit_ramp);
		
		gim.sensor.yaw_offset_angle   = gyro_data.yaw;
	}
	else if((gim.ctrl_mode == GIMBAL_NORMAL) || (gim.ctrl_mode == GIMBAL_WRITHE))
	{
		gim.sensor.yaw_relative_angle = gyro_data.yaw   - gim.sensor.yaw_offset_angle;
		gim.sensor.pit_relative_angle = gyro_data.pitch - gim.sensor.pit_offset_angle;//gim.sensor.pit_offset_angleÔÝÊ±Îª0
		
		pid_yaw.p = 5;

		ramp_init(&pit_ramp, PIT_PREPARE_TIMS_MS);
		ramp_init(&yaw_ramp, YAW_PREPARE_TIMS_MS);
	}
	/* get gimbal relative palstance */
  //the Z axis(yaw) of gimbal coordinate system corresponds to the IMU Z axis
  gim.sensor.yaw_palstance = gyro_data.v_z;
  //the Y axis(pitch) of gimbal coordinate system corresponds to the IMU -X axis
  gim.sensor.pit_palstance = gyro_data.v_x;
	/* get remote and keyboard gimbal control information */
  remote_ctrl_gimbal_hook();
	GimbalAngleLimit();
  osDelay(1);
}
/********************************* GIMBAL REMOTE  HANDLER ***************************************/
void remote_ctrl_gimbal_hook(void)
{
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == REMOTE_CTRL))
		{					
			/* get remote gimbal info */
		  gim.pid.yaw_angle_ref -= RC_CtrlData.rc.ch2 * GIMBAL_YAW_REF_FACT;
			gim.pid.pit_angle_ref -= RC_CtrlData.rc.ch3 * GIMBAL_PIT_REF_FACT;
		}
}
void GimbalAngleLimit(void)
{
	if(ramp_mode == RAMP_UP)
	{
		VAL_LIMIT(gim.pid.pit_angle_ref, -50 ,-5);  
	}
	else if(ramp_mode == RAMP_DOWN)
	{
		VAL_LIMIT(gim.pid.pit_angle_ref, -50 ,30);  
	}
	else
	{
		VAL_LIMIT(gim.pid.pit_angle_ref, -35 ,12);
	}
	VAL_LIMIT(gim.pid.yaw_angle_ref, gim.sensor.yaw_relative_angle-30, gim.sensor.yaw_relative_angle+30);  
}
void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[])
{
  /* 0: yaw motor current
     1: pitch motor current
     2: trigger motor current*/
  send_gimbal_cur(-gimbal_cur[0], gimbal_cur[1], gimbal_cur[2]);
}
