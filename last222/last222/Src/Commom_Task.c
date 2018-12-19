#include "main.h"

/* communicate task global parameter */
motor_current_t glb_cur;

void Can_Send_Task(void const *argument)
{
		osEvent event;
	for(;;)
	{
		event = osSignalWait(GIMBAL_MOTOR_MSG_SEND   | \
                         CHASSIS_MOTOR_MSG_SEND |
													SHOOT_MOTOR_MSG_SEND, osWaitForever);
		 if (event.status == osEventSignal)
    {
      if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
      //send_chassis_motor_ctrl_message(glb_cur.chassis_cur);
      }
			else if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
      {
			send_gimbal_motor_ctrl_message(glb_cur.gimbal_cur);     
      }
			else if (event.value.signals & SHOOT_MOTOR_MSG_SEND)
			{
			send_shoot_motor_ctrl_message(glb_cur.shoot_cur);
			}
    }
	}
}
