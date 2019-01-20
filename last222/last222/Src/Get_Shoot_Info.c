#include "main.h"
extern uint8_t auto_shoot;
extern uint32_t turn_time_last;
void Get_Shoot_Info(void const * argument)
{
	osEvent event;
  for(;;)
  {
		event = osSignalWait(SHOOT_GET_SIGNAL,osWaitForever);
		if (event.status == osEventSignal)
	 {
			if(event.value.signals & SHOOT_GET_SIGNAL)
	   {
				taskENTER_CRITICAL();
			  shoot.trig.trig_spd 				  = moto_trigger.speed_rpm * 0.10472/ 36;//W=2*PI*N
				/* get remote and keyboard trig and friction wheel control information */
				remote_ctrl_shoot_hook();
				keyboard_shoot_hook();
				taskEXIT_CRITICAL();
     }
   } 
 }
}
void send_shoot_motor_ctrl_message(int16_t shoot_cur[])
{
  /* 0: up friction wheel motor current
     1: down friction wheel motor current
     2: trigger motor current*/
  send_shoot_cur(shoot_cur[0], shoot_cur[1], shoot_cur[2]);
}
void remote_ctrl_shoot_hook(void)
{	
	if(ctrl_mode == REMOTE_CTRL)
	{
		shoot.fric.switching = RC_CtrlData.rc.s1;
		shoot.mode = RC_CtrlData.rc.s2;
	}
}
void keyboard_shoot_hook(void)
{
	if((RC_CtrlData.key.v & F_KEY) && (handler_run_time -turn_time_last>350))																
	{
		turn_time_last = handler_run_time;
		
		if(shoot.fric.switching == ON)
			shoot.fric.switching  = OFF;
		else
			shoot.fric.switching  = ON;
	}
	
	if((RC_CtrlData.mouse.press_l == 1) && (shoot.fric.switching == ON))
	{
		shoot.trig.switching = ON;
	}
	else
	{
		shoot.trig.switching = OFF;
	}
	
}
