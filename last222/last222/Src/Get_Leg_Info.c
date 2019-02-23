#include "main.h"
extern ramp_t LegSpeedRamp;
uint8_t qudan_jindu=1;
uint8_t CTRL_OPEN;
uint32_t last_run_time;
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
	if(ctrl_mode == REMOTE_CTRL && leg_mode == leg_normal_mode)
	{
			chassis_leg.vx =  RC_CtrlData.rc.ch3 * CHASSIS_REF_FACT;
	}
}
void keyboard_leg_hook(void)
{
	if(ctrl_mode == KEYBOR_CTRL && leg_mode == leg_normal_mode)
	{
		//�ж��Ƿ���CTRL��
		if(RC_CtrlData.key.v & CTRL_KEY)
		{
			CTRL_OPEN = 1;
		}
		else
		{
		  CTRL_OPEN =0;
		}
		if(RC_CtrlData.mouse.press_l == 1 && (handle_run_time - last_run_time)>=1000)
		{

		}
			/* ������������*/
		if(RC_CtrlData.key.v & R_KEY)
		{
			chassis_leg.vx = 20;
		}
		else if(RC_CtrlData.key.v & F_KEY)
		{
			chassis_leg.vx = -20;
		}
		else
		{
			chassis_leg.vx = 0;
		}
			/* ǰ����������*/

		if( CTRL_OPEN && (RC_CtrlData.key.v & Q_KEY))
		{
			chassis_leg.vy_foreword_leg = -20;
		}
		else if(RC_CtrlData.key.v & Q_KEY )
		{
			chassis_leg.vy_foreword_leg = 20;
		}
		else 
		{
			chassis_leg.vy_foreword_leg = 0;
		}
			/* ������������*/
		if( CTRL_OPEN && (RC_CtrlData.key.v & E_KEY))
		{
			chassis_leg.vy_back_leg = -20;
		}
		else if(RC_CtrlData.key.v & E_KEY )
		{
			chassis_leg.vy_back_leg = 20;
		}
		else 
		{
			chassis_leg.vy_back_leg = 0;
		}
			//	��ط�����;
		if((RC_CtrlData.key.v & Z_KEY) && (handle_run_time - last_run_time)>=500)
		{	//�м���
			last_run_time=handle_run_time;
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_7);
		}
		else if((RC_CtrlData.key.v & X_KEY) && (handle_run_time - last_run_time)>=500)
		{	//�ϳ�
			last_run_time=handle_run_time;
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
		}
		else if((RC_CtrlData.key.v & C_KEY) && (handle_run_time - last_run_time)>=500)
		{
			//CATCH
			last_run_time=handle_run_time;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1);
		}
		else	if((RC_CtrlData.key.v & V_KEY) && (handle_run_time - last_run_time)>=500)
		{
			//PUSH
			last_run_time=handle_run_time;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
		}
		else	if((RC_CtrlData.key.v & B_KEY) && (handle_run_time - last_run_time)>=500)
		{//����
			last_run_time=handle_run_time;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);
		}
	}
}
