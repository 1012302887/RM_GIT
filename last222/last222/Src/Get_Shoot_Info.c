#include "main.h"
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
			shoot.trig.trig_spd 				  = moto_trigger.filter_rate / 36;
			shoot.trig.trig_pos        		= moto_trigger.total_angle / 36;
				/* get friction wheel fdb speed */
				for (uint8_t i = 0; i < 2; i++)
				{
					shoot.fric.fric_wheel_spd_fdb[i] = moto_friction[i].filter_rate * (6.2832 / 8.192);
				}
				/* get remote and keyboard trig and friction wheel control information */
				remote_ctrl_shoot_hook();
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
	if(shoot.mode == REMOTE_CTRL)
	{
		shoot.fric.switching = RC_CtrlData.rc.s1;
		shoot.mode = RC_CtrlData.rc.s2;
	}
	else if(shoot.mode != SHOOT_BUFF)
	{
		shoot.mode = RC_CtrlData.rc.s2;
	}		
}
