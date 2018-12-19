#include "main.h"
extern ramp_t pit_ramp;
extern ramp_t yaw_ramp;
extern uint8_t auto_shoot;
extern uint32_t turn_time_last;
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
	keyboard_gimbal_hook();
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
static float add_angle = 0;
uint32_t pc_i = 0; 
void keyboard_gimbal_hook(void)
{	
	if((gim.ctrl_mode != GIMBAL_INIT) && (ctrl_mode == KEYBOR_CTRL))
	{
		VAL_LIMIT(RC_CtrlData.mouse.x, -128, 128); 
		VAL_LIMIT(RC_CtrlData.mouse.y, -18, 32);   
		/* get remote gimbal info */
		pc_i++;
		pc_data.yaw_befoer[pc_i%100] = gim.sensor.yaw_relative_angle; 
		if(auto_shoot == 1)
		{	
			pid_yaw.p = 20;
			if(RC_CtrlData.key.v & Q )      {add_angle += 0.01f;}
			else if(RC_CtrlData.key.v & E ) {add_angle -= 0.01f;}
			else {add_angle = 0;}
			gim.pid.yaw_angle_ref  = pc_data.filter_yaw-RC_CtrlData.mouse.x*0.2f+add_angle;	
			gim.pid.pit_angle_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT ;
		}
		else if(gim.ctrl_mode == GIMBAL_SUPPLY)
		{
			gim.pid.yaw_angle_ref	 = 0;
			gim.pid.pit_angle_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		}
		else if(gim.ctrl_mode == GIMBAL_SHOOT_BUFF)
		{
//			RC_CtrlData.mouse.x = 0;
//			RC_CtrlData.mouse.y = 0;
		}
		else 
		{
			pc_data.v_now_i = 0;
			pid_yaw.p = 10;
			gim.pid.yaw_angle_ref -= RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT  ;//+ shoot_buff_data.dynamic_yaw * 0.015f;
			gim.pid.pit_angle_ref += RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT ;//+ shoot_buff_data.dynamic_pit * 0.015f;
		}
	}	
	else
	{
		RC_CtrlData.mouse.x = 0;
		RC_CtrlData.mouse.y = 0;
	}
		     
	/* car is or is not supply state */
  if((RC_CtrlData.key.v & Z) && (handler_run_time -turn_time_last>350))	// Gimbal_Task "handler_run_time++"												
	{
		turn_time_last = handler_run_time;
		
		if(gim.ctrl_mode == GIMBAL_NORMAL)
		{	
			HATCH_COVER   = OPEN_COVER;
			gim.ctrl_mode = GIMBAL_SUPPLY;
		}
		else
		{
			HATCH_COVER   = CLOSE_COVER;
			gim.ctrl_mode = GIMBAL_NORMAL;
		}
	}
	
	/* car is or is not shoot_buff state */
	if((RC_CtrlData.key.v & R) && (handler_run_time -turn_time_last>350))																
	{
		turn_time_last = handler_run_time;
		
		if(gim.ctrl_mode == GIMBAL_NORMAL)
		{	
			gim.ctrl_mode = GIMBAL_SHOOT_BUFF;
			shoot.mode    = SHOOT_BUFF;
			pc_data.shoot_data = 1;
		}
		else
		{
			gim.ctrl_mode = GIMBAL_NORMAL;
			shoot.mode    = KEYBOR_CTRL;
			pc_data.shoot_data = 0;
		}
	}
	
	/* car is or is not autushoot */
	if(RC_CtrlData.mouse.press_r == 1)
	{
		auto_shoot = 1;
	}
	else
	{
		auto_shoot = 0;
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
