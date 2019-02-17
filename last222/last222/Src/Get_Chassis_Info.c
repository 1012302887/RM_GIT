#include "main.h"
uint32_t turn_time_last = 0;
extern ramp_t FBSpeedRamp;
extern ramp_t LRSpeedRamp;
extern float dipan_fdb_KF[4];
static float forward_back_speed = 0;static float left_right_speed = 0;
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
//				chassis.follow_gimbal =gyro_data.yaw_angle;  	 
				/* get chassis wheel fdb speed */
				for (uint8_t i = 0; i < 4; i++)
				 {
//						chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm*0.10472/19.0;//W=2*PI*N
					 chassis.wheel_spd_fdb[i] = moto_chassis[i+4].speed_rpm*0.0055116;//W=2*PI*N
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
	if(ctrl_mode == REMOTE_CTRL)
	{
		/* get chassis wheel ref speed */
			chassis.vx_offset = RC_CtrlData.rc.ch1 * CHASSIS_REF_FACT;
			chassis.vy_offset = RC_CtrlData.rc.ch0 * CHASSIS_REF_FACT;
			chassis.vw_offset -= RC_CtrlData.rc.ch2 * 0.0004f;
	}
}

void keyboard_chassis_hook(void)
{
	VAL_LIMIT(RC_CtrlData.mouse.x, -128, 128); 
	VAL_LIMIT(RC_CtrlData.mouse.y, -18, 32); 
	if(ctrl_mode == KEYBOR_CTRL)
	{
		/*********** speed mode: normal speed/high speed ***********/                   
		if(RC_CtrlData.key.v & SHIFT_KEY)																	
		{
			forward_back_speed = HIGH_FORWARD_BACK_SPEED;
			left_right_speed 	 = HIGH_LEFT_RIGHT_SPEED;
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
			//¿ØÖÆµ×ÅÌ×ªÍä
		if(RC_CtrlData.key.v & Z_KEY)
		{
				chassis.vw_offset -= RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT  ;
				chassis.vw_offset += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT ;
		}
			//¼ÌµçÆ÷¿ØÖÆµç³Ø·§
		if(RC_CtrlData.key.v & V_KEY)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		}
		else if(RC_CtrlData.key.v & B_KEY)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		}
	}	
}


void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
//	   send_chassis_cur(chassis_cur[0] * chassis.power_surplus, chassis_cur[1] *chassis.power_surplus, 
//		 chassis_cur[2] * chassis.power_surplus, chassis_cur[3] *chassis.power_surplus);
	send_chassis_cur(chassis_cur[0] , chassis_cur[1] , 
										chassis_cur[2] , chassis_cur[3]);
}

