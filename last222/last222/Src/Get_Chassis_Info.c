#include "main.h"

/*left wheel:2   right wheel:1 */

/*left wheel:4   right wheel:4 */

/* USER CODE END Header_Get_Chassis_Info */
void Get_Chassis_Info(void const * argument)
{
	osEvent event;
  for(;;)
  {
		event = osSignalWait(INFO_GET_SIGNAL,osWaitForever);
		if(event.value.signals & INFO_GET_SIGNAL)
	   {
					/* get gimbal and chassis relative angle */
				chassis.follow_gimbal = moto_yaw.total_angle;
				/* get chassis wheel fdb positin */
				for (uint8_t i = 0; i < 4; i++)
				 {
					chassis.wheel_pos_fdb[i] = moto_chassis[i].total_angle/19.0f;
				 }
				/* get chassis wheel fdb speed */
				for (uint8_t i = 0; i < 4; i++)
				 {
					chassis.wheel_spd_fdb[i] = moto_chassis[i].filter_rate/19.0;
				 }
				/* get remote and keyboard chassis control information */
				remote_ctrl_chassis_hook();
     }
		 }
}
/******************************** CHASSIS REMOTE  HANDLER ***************************************/

void remote_ctrl_chassis_hook(void)
{
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == REMOTE_CTRL))
	{
		/* get chassis wheel ref speed */
		if(ramp_mode == RAMP_UP)
		{
			chassis.vx_offset = RC_CtrlData.rc.ch1 * 30.0f / 660;
			chassis.vy_offset = RC_CtrlData.rc.ch0 * 30.0f / 660;
		}
		else
		{
			chassis.vx_offset = RC_CtrlData.rc.ch1 * CHASSIS_REF_FACT;
			chassis.vy_offset = RC_CtrlData.rc.ch0 * CHASSIS_REF_FACT;
		}
	}
}
void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
	 //send_chassis_cur(chassis_cur[0] * chassis.power_surplus, chassis_cur[1] *chassis.power_surplus, 
										//chassis_cur[2] * chassis.power_surplus, chassis_cur[3] *chassis.power_surplus);
	send_chassis_cur(chassis_cur[0] , chassis_cur[1] , 
										chassis_cur[2] , chassis_cur[3]);
}
