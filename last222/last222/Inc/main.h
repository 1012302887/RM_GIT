#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_tim.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "math.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include "Mode_Switch_Task.h"
#include "Ramp.h"
#include "Can_Send_Task.h"
#include "Get_Gimbal_Info.h"
#include "Get_Chassis_Info.h"
#include "Can.h"
#include "Usart.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "PID.h"
#include "Common_Task.h"
#include "NI_MING.h"
#include "Get_Shoot_Info.h"
#include "Shoot_Task.h"
#include "Keyboard.h"
#include "Kalman.h"
#include "REG_DMA.h"
#include "JUDG_INFO.h"
#include "first_order_filter.h"
#include "trcConfig.h"
#include "Time.h"
/* USER CODE BEGIN Includes */
#define       CAR_NUM      1

#if   CAR_NUM == 1
  #define				AUTOSHOOT_X_OFFSET  -2
	#define       MOTO_YAW_OFFSET_ECD  150
	#define       MOTO_PIT_OFFSET_ECD  1800
#elif CAR_NUM == 4
  #define				AUTOSHOOT_X_OFFSET  -2
	#define       MOTO_YAW_OFFSET_ECD  3049
	#define       MOTO_PIT_OFFSET_ECD  847
#endif

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
