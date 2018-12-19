#include "main.h"
#include "math.h"

/* shoot task global parameter */
shoot_t shoot = {0};

extern osThreadId CAN_SEND_TASKHandle;
extern osThreadId GET_SHOOT_TASK_HANDEL;
extern uint32_t shoot_time;
uint32_t shoot_time_last;
int shoot_time_ms;
//static float pidin, ppidp, ppidi, ppidd, spidp, spidi, spidd = 0
/* �����ʱ����*/
void Shoot_Task(void const * argument)
{
    if(gim.ctrl_mode != GIMBAL_INIT)
    {
				 switch (shoot.mode)
				{
					case REMOTE_CTRL:
						shoot_remote_handler();
					break;
					case KEYBOR_CTRL:
						shoot_keyboard_handler();
						pid_fric_spd[0].i = 0;
						pid_fric_spd[1].i = 0;
					break;
				 default:
					break;
				}
				if(shoot.heat > shoot.trig.max_ref)
				{shoot.trig.spd_ref = 0;}
				
				glb_cur.shoot_cur[0] = chassis_pid_calc(&pid_fric_spd[0], shoot.fric.fric_wheel_spd_fdb[0], -shoot.fric.fric_wheel_spd_ref[0] * 28.571248f);
				glb_cur.shoot_cur[1] = chassis_pid_calc(&pid_fric_spd[1], shoot.fric.fric_wheel_spd_fdb[1],  shoot.fric.fric_wheel_spd_ref[1] * 28.571248f);
				glb_cur.shoot_cur[2] = chassis_pid_calc(&pid_trigger_spd, shoot.trig.trig_spd, shoot.trig.spd_ref);				
				osSignalSet(GET_SHOOT_TASK_HANDEL, SHOOT_GET_SIGNAL);
				osSignalSet(CAN_SEND_TASKHandle, SHOOT_MOTOR_MSG_SEND);
		}
}

void shoot_remote_handler(void)
{
	if(shoot.fric.switching == ON)
	{
		shoot.trig.delay++;
		shoot.trig.max_ref = 88888;
		shoot.fric.fric_wheel_spd_ref[0] = shoot.fric.fric_wheel_spd_ref[1] = 15;		
		if(shoot.trig.delay > 1000)
		{
			shoot.trig.spd_ref = -6;
		}
	}
	else
	{	
		shoot.trig.delay = 0;
		shoot.trig.spd_ref = 0;
		shoot.fric.fric_wheel_spd_ref[0] = shoot.fric.fric_wheel_spd_ref[1] = 0;	
	}
}

void shoot_keyboard_handler(void)
{
	if(shoot.fric.switching == ON)
	{
		if(RC_CtrlData.key.v & B)
		{
			shoot.fric.fric_wheel_spd_ref[0] = shoot.fric.fric_wheel_spd_ref[1] = 16;
		}
		else
		{
			shoot.fric.fric_wheel_spd_ref[0] = shoot.fric.fric_wheel_spd_ref[1] = 14;
		}
	}
	else
	{
		shoot.fric.fric_wheel_spd_ref[0] = shoot.fric.fric_wheel_spd_ref[1] = 0;
	}
	
	if(shoot.trig.switching == ON)
	{
		switch (shoot.level)
	  {		
		 case Lv_3:
			 if(shoot.buff & FIVEFOLD_HEAT)
			 {
				 shoot.trig.spd_ref = -15;
			 }
			 else
			 {
				shoot.trig.spd_ref = -6;
			 }				 
		 break;
		
		 case Lv_2:
			 shoot.trig.max_ref = 210;
			 if(shoot.buff & FIVEFOLD_HEAT)
			 {
				 shoot.trig.spd_ref = -10;
			 }
			 else
			 {
				 shoot.trig.spd_ref = -4;  //4
			 }
		 break;
		
		 default:
			 shoot.trig.max_ref = 100;
			 if(shoot.buff & FIVEFOLD_HEAT)
			 {
				 shoot.trig.spd_ref = -6;
			 }
			 else
			 {
				 shoot.trig.spd_ref = -2; //2~3
			 }
		 break;
	  }
	}
	else
	{
		shoot.trig.spd_ref = 0;
	}
	shoot.get_pos = 1;
}
void Shoot_Param_Init(void)
{
		 for (int k = 0; k < 2; k++)
  {
    PID_struct_init(&pid_fric_spd[k], POSITION_PID, 10000, 7000,
		150, 3, 0); 
	}
	
	shoot.trig.max_ref = 100;
	 /* bullet trigger motor pid parameter */
  PID_struct_init(&pid_trigger, POSITION_PID, 30, 2000,
                  0.5, 0, 0);
  PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 5000,
                  1200, 2, 0);
}
