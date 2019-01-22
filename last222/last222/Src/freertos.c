/* Includes ------------------------------------------------------------------*/
#include "main.h"
osThreadId defaultTaskHandle;
osThreadId CAN_SEND_TASKHandle;
osThreadId MODE_SWITCH_TASKHandle;
osThreadId GET_GIMBAL_INFOHandle;
osThreadId GET_CHASSIS_INFHandle;
osThreadId GET_SHOOT_TASK_HANDEL;
osTimerId chassis_timer_id;
osTimerId gimbal_timer_id;
osTimerId shoot_timer_id;
void StartDefaultTask(void const * argument);
/* USER CODE END Includes */
void FreeRtos_Init(void)
{
	  taskENTER_CRITICAL();
	
	  osTimerDef(chassisTimer, Chassis_Task);
		chassis_timer_id = osTimerCreate(osTimer(chassisTimer), osTimerPeriodic, NULL);
	
		osTimerDef(gimTimer, Gimbal_Task);
		gimbal_timer_id =  osTimerCreate(osTimer(gimTimer),   osTimerPeriodic, NULL);
	
	  osTimerDef(shootTimer, Shoot_Task);
		shoot_timer_id =  osTimerCreate(osTimer(shootTimer), osTimerPeriodic, NULL);
	
	
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CAN_SEND_TASK */
    osThreadDef(CAN_SEND_TASK, Can_Send_Task, osPriorityAboveNormal, 0, 256);
    CAN_SEND_TASKHandle = osThreadCreate(osThread(CAN_SEND_TASK), NULL);

  /* definition and creation of MODE_SWITCH_TAS */
    osThreadDef(MODE_SWITCH_TAS, Mode_Switch_Task, osPriorityHigh, 0, 256);
    MODE_SWITCH_TASKHandle = osThreadCreate(osThread(MODE_SWITCH_TAS), NULL);

  /* definition and creation of GET_GIMBAL_INFO */
    osThreadDef(GET_GIMBAL_INFO, Get_Gimbal_Info, osPriorityAboveNormal, 0, 512);
    GET_GIMBAL_INFOHandle = osThreadCreate(osThread(GET_GIMBAL_INFO), NULL);

  /* definition and creation of GET_CHASSIS_INF */
    osThreadDef(GET_CHASSIS_INF, Get_Chassis_Info, osPriorityAboveNormal, 0, 512);
    GET_CHASSIS_INFHandle = osThreadCreate(osThread(GET_CHASSIS_INF), NULL);
		
  /* definition and creation of GET_CHASSIS_INF */
    osThreadDef(GET_SHOOT_INF, Get_Shoot_Info, osPriorityAboveNormal, 0, 256);
    GET_SHOOT_TASK_HANDEL = osThreadCreate(osThread(GET_SHOOT_INF), NULL);
		
	taskEXIT_CRITICAL();
}

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
