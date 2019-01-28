#include "main.h"

/* communicate task global parameter */
motor_current_t glb_cur;

void Can_Send_Task(void const *argument)
{
		osEvent event;
	for(;;)
	{
		event = osSignalWait(CHASSIS_MOTOR_MSG_SEND| LEG_MOTOR_MSG_SEND,osWaitForever);
		 if (event.status == osEventSignal)
    {
      if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
      send_chassis_motor_ctrl_message(glb_cur.chassis_cur);
      }
			 if (event.value.signals & LEG_MOTOR_MSG_SEND)
      {
      send_leg_cur(chassis_leg.current[0],chassis_leg.current[1],chassis_leg.current[2],chassis_leg.current[3]);
      }
    }
	}
}
