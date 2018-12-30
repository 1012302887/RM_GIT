#include "main.h"
#include "math.h"
ramp_t fric_ramp 	 = RAMP_GEN_DAFAULT;
uint8_t fric_swich=0;
/* shoot task global parameter */
shoot_t shoot = {0};

extern osThreadId CAN_SEND_TASKHandle;
extern osThreadId GET_SHOOT_TASK_HANDEL;
extern uint32_t shoot_time;
uint32_t shoot_time_last;
int shoot_time_ms;
//static float pidin, ppidp, ppidi, ppidd, spidp, spidi, spidd = 0
/* 射击定时任务*/
void Shoot_Task(void const * argument)
{
    if(gim.ctrl_mode != GIMBAL_INIT)
    {
				 switch (ctrl_mode)
				{
					case REMOTE_CTRL:
						shoot_remote_handler();
					break;
					case KEYBOR_CTRL:
						shoot_keyboard_handler();
					break;
				 default:
					break;
				}
				glb_cur.shoot_cur[0] = 0;
				glb_cur.shoot_cur[1] = 0;
				glb_cur.shoot_cur[2] = chassis_pid_calc(&pid_trigger_spd, shoot.trig.trig_spd, -shoot.trig.spd_ref);
				osSignalSet(GET_SHOOT_TASK_HANDEL, SHOOT_GET_SIGNAL);
				osSignalSet(CAN_SEND_TASKHandle, GIMBAL_MOTOR_MSG_SEND);
//				osSignalSet(CAN_SEND_TASKHandle, SHOOT_MOTOR_MSG_SEND);
		}
}

void shoot_remote_handler(void)
{
	if(shoot.fric.switching == ON)
	{		
		shoot.trig.delay++;
		send_fri_cur(4400*ramp_calc(&fric_ramp));
			if(shoot.trig.delay > 500)
		{
			shoot.trig.spd_ref = -6;
		}
	}
	else
	{	

		shoot.trig.delay = 0;
		shoot.trig.spd_ref = 0;
		ramp_init(&fric_ramp,FRIC_PREPARE_TIMS_MS);
		send_fri_cur(3000*ramp_calc(&fric_ramp));
	}
}

void shoot_keyboard_handler(void)
{
	if(shoot.fric.switching == ON)
	{
		if(RC_CtrlData.key.v & B_KEY)
		{
				send_fri_cur(4800*ramp_calc(&fric_ramp));
		}
		else
		{
				send_fri_cur(4400*ramp_calc(&fric_ramp));
		}
	}
	else
	{
		ramp_init(&fric_ramp,FRIC_PREPARE_TIMS_MS);
		send_fri_cur(3000*ramp_calc(&fric_ramp));
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
	shoot.trig.max_ref = 100;
	 /* bullet trigger motor pid parameter */
  PID_struct_init(&pid_trigger, POSITION_PID, 30, 2000,
                  0.5, 0, 0);
  PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 5000,
	100, 2, 0);//p:1200
	PID_struct_init(&pid_as_yaw, POSITION_PID, 10000, 5000,
	100, 2, 0);//
	ramp_init(&fric_ramp,FRIC_PREPARE_TIMS_MS);
}
