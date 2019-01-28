#include "main.h"
extern ramp_t LegSpeedRamp;
void Get_Leg_Info(void const * argument)
{
	osEvent event;
  for(;;)
  {
		event = osSignalWait(LEG_INFO_GET_SIGNAL,osWaitForever);
		if(event.value.signals & LEG_INFO_GET_SIGNAL)
	   {	
			 taskENTER_CRITICAL();
				/* get leg wheel fdb speed */
				for (uint8_t i = 0; i < 4; i++)
				 {
//					 chassis_leg.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm*0.003879;//W=2*PI*N
					 chassis_leg.wheel_spd_fdb[i] = moto_chassis[i].filter_rate/27;
				 }
				/* get remote and keyboard leg control information */
				if(leg_mode != leg_init_mode)
				{
					keyboard_leg_hook();
					remote_ctrl_leg_hook();
				}
				taskEXIT_CRITICAL();	 
     }
	}
}
/******************************** CHASSIS REMOTE  HANDLER ***************************************/

void remote_ctrl_leg_hook(void)
{
	if(ctrl_mode == REMOTE_CTRL&&leg_mode == leg_normal_mode)
	{
			if (RC_CtrlData.rc.s1==1&&RC_CtrlData.rc.s2==1)
				{

				}
			else if(RC_CtrlData.rc.s1==2&&RC_CtrlData.rc.s2==1)
				{

				}
			else
				{
					
				}
			chassis_leg.vx =  RC_CtrlData.rc.ch3 * CHASSIS_REF_FACT;
			chassis_leg.vy =  RC_CtrlData.rc.ch2 * CHASSIS_REF_FACT;
	}
}
void keyboard_leg_hook(void)
{

}
