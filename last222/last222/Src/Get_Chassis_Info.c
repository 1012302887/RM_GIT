#include "main.h"
uint8_t auto_shoot = 0;
uint32_t turn_time_last = 0;
extern ramp_t FBSpeedRamp;
extern ramp_t LRSpeedRamp;
extern float dipan_fdb_KF[4];
/*left wheel:2   right wheel:1 */

/*left wheel:4   right wheel:3 */

/* USER CODE END Header_Get_Chassis_Info */
void Get_Chassis_Info(void const * argument)
{
	osEvent event;
  for(;;)
  {
		event = osSignalWait(INFO_GET_SIGNAL,osWaitForever);
		if(event.value.signals & INFO_GET_SIGNAL)
	   {	
			 taskENTER_CRITICAL();
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
//					 /*ÂË²¨*/
						chassis.wheel_spd_fdb[i] =	Kalman_filter_calc(&CHASSIS_KF[i],chassis.wheel_spd_fdb[i]);
//					 /*ÂË²¨*/
				 }
				 
				 if((InfantryJudge.remainPower > 60) || (InfantryJudge.remainPower < 0))
				 {InfantryJudge.remainPower = 0;}
				chassis.power_surplus = InfantryJudge.remainPower / 60;
				 
				/* get remote and keyboard chassis control information */
				keyboard_chassis_hook();
				remote_ctrl_chassis_hook();
				 taskEXIT_CRITICAL();
				 
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

void keyboard_chassis_hook(void)
{
	static float forward_back_speed = 0;
	static float left_right_speed 	 = 0;
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == KEYBOR_CTRL))
	{
		/*********** speed mode: normal speed/high speed ***********/                   
		if(ramp_mode == RAMP_UP)
		{
			forward_back_speed = 30.0f;
			left_right_speed   = 30.0f;
		}
		else if(RC_CtrlData.key.v & SHIFT_KEY)																	
		{
			forward_back_speed = HIGH_FORWARD_BACK_SPEED;
			left_right_speed 	 = HIGH_LEFT_RIGHT_SPEED;
		}
		else if(gim.ctrl_mode == GIMBAL_WRITHE)
		{
			forward_back_speed = 40.0f;
			left_right_speed 	 = 30.0f;
		}
		else
		{
			forward_back_speed = NORMAL_FORWARD_BACK_SPEED;
			left_right_speed   = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		/*********** get forward chassis wheel ref speed ***********/
		if(RC_CtrlData.key.v & W_KEY)
		{
			chassis.vx_offset =  forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else if(RC_CtrlData.key.v & S_KEY)
		{
			chassis.vx_offset = -forward_back_speed * ramp_calc(&FBSpeedRamp);
		}
		else
		{
			chassis.vx_offset = 0;
			ramp_init(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
		}
		
		/*********** get rightward chassis wheel ref speed ***********/
		if(RC_CtrlData.key.v & A_KEY)
		{
			chassis.vy_offset = -left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else if(RC_CtrlData.key.v & D_KEY)
		{
			chassis.vy_offset = left_right_speed * ramp_calc(&LRSpeedRamp);
		}
		else
		{
			chassis.vy_offset = 0;
			ramp_init(&LRSpeedRamp, MOUSR_LR_RAMP_TICK_COUNT);
		}
		
		/*********** get chassis rotate ref ***********/
		if(RC_CtrlData.key.v & Q_KEY && auto_shoot != 1)
		{
			if(gim.ctrl_mode == GIMBAL_SUPPLY)
			{
				chassis.vw_offset = -30.0f ;//* ramp_calc(&LRSpeedRamp);;
			}
			else
			{
				gim.pid.yaw_angle_ref += 
				CHASSIS_ROTATE_FACT;
			}
		}
		else if(RC_CtrlData.key.v & E_KEY && auto_shoot != 1)
		{
			if(gim.ctrl_mode == GIMBAL_SUPPLY)
			{
				chassis.vw_offset = 30.0f ;
			}
			else
			{
					gim.pid.yaw_angle_ref -= CHASSIS_ROTATE_FACT;
			}
		}
		else
		{
			chassis.vw_offset = 0;
		}
		
		/*********** chassis mode is waist ***********/
		if((RC_CtrlData.key.v & CTRL_KEY) && (handler_run_time -turn_time_last>350)) // Gimbal_Task "handler_run_time++"													
		{
			turn_time_last = handler_run_time;
			
			if(gim.ctrl_mode == GIMBAL_NORMAL)
			{	
				gim.ctrl_mode = GIMBAL_WRITHE;
			}
			else
			{
				gim.ctrl_mode = GIMBAL_NORMAL;
			}
		}
		
	}
	
}


void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
	   send_chassis_cur(chassis_cur[0] * chassis.power_surplus, chassis_cur[1] *chassis.power_surplus, 
		 chassis_cur[2] * chassis.power_surplus, chassis_cur[3] *chassis.power_surplus);
//	send_chassis_cur(chassis_cur[0] , chassis_cur[1] , 
//										chassis_cur[2] , chassis_cur[3]);
}

