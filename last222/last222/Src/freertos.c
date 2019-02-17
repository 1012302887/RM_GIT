/* Includes ------------------------------------------------------------------*/
#include "main.h"
osThreadId defaultTaskHandle;
osThreadId CAN_SEND_TASKHandle;
osThreadId MODE_SWITCH_TASKHandle;
osThreadId GET_CHASSIS_INFHandle;
osTimerId chassis_timer_id;
void StartDefaultTask(void const * argument);
/* USER CODE END Includes */
void FreeRtos_Init(void)
{
	  taskENTER_CRITICAL();
	
	  osTimerDef(chassisTimer, Chassis_Task);
		chassis_timer_id = osTimerCreate(osTimer(chassisTimer), osTimerPeriodic, NULL);
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
  /* definition and creation of CAN_SEND_TASK */
    osThreadDef(CAN_SEND_TASK, Can_Send_Task, osPriorityAboveNormal, 0, 256);
    CAN_SEND_TASKHandle = osThreadCreate(osThread(CAN_SEND_TASK), NULL);

  /* definition and creation of MODE_SWITCH_TAS */
    osThreadDef(MODE_SWITCH_TAS, Mode_Switch_Task, osPriorityAboveNormal, 0, 256);
    MODE_SWITCH_TASKHandle = osThreadCreate(osThread(MODE_SWITCH_TAS), NULL);

  /* definition and creation of GET_CHASSIS_INF */
    osThreadDef(GET_CHASSIS_INF, Get_Chassis_Info, osPriorityAboveNormal, 0, 512);
    GET_CHASSIS_INFHandle = osThreadCreate(osThread(GET_CHASSIS_INF), NULL);
		
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
