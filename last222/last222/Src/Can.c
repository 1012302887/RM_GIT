#include "main.h"
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef FILTER1;
CAN_FilterTypeDef FILTER2;

CAN_TxHeaderTypeDef    Tx1Message;      
CAN_RxHeaderTypeDef    Rx1Message;  
CAN_TxHeaderTypeDef    Tx2Message;      
CAN_RxHeaderTypeDef    Rx2Message;

moto_measure_t moto_pit;//储存pit轴电机信息的结构体
moto_measure_t moto_yaw;//储存yaw轴电机信息的结构体
moto_measure_t moto_chassis[4];//储存底盘四个电机信息的结构体
shoot_moto_measure_t moto_friction[2];//储存拨盘电机信息的结构体
moto_measure_t moto_trigger;//储存拨盘电机信息的结构体
GYRO_DATA gyro_data;//储存陀螺仪信息的结构体
uint8_t DATA[8];//储存CAN1接收到数据
uint8_t Data[8];//储存CAN2接收到数据
static uint8_t i;
/* CAN1 init function */
void MX_CAN1_Init(void)
{
	//printf("CAN1__INIT");
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	FILTER1.FilterIdHigh=0X0000;     //32位ID
  FILTER1.FilterIdLow=0X0000;
  FILTER1.FilterMaskIdHigh=0X0000; //32位MASK
  FILTER1.FilterMaskIdLow=0X0000;  
  FILTER1.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  FILTER1.FilterBank=0;          //过滤器0
  FILTER1.FilterMode=CAN_FILTERMODE_IDMASK;
  FILTER1.FilterScale=CAN_FILTERSCALE_32BIT;
  FILTER1.FilterActivation=ENABLE; //激活滤波器0
  FILTER1.SlaveStartFilterBank=14;
  HAL_CAN_ConfigFilter(&hcan1,&FILTER1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* CAN2 init function */
void MX_CAN2_Init(void)
{
	//printf("CAN2__INIT");
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	FILTER2.FilterIdHigh=0X0000;     //32位ID
  FILTER2.FilterIdLow=0X0000;
  FILTER2.FilterMaskIdHigh=0X0000; //32位MASK
  FILTER2.FilterMaskIdLow=0X0000;  
  FILTER2.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  FILTER2.FilterBank=14;
  FILTER2.FilterMode=CAN_FILTERMODE_IDMASK;
  FILTER2.FilterScale=CAN_FILTERSCALE_32BIT;
  FILTER2.FilterActivation=ENABLE; 
  FILTER2.SlaveStartFilterBank=14;
  HAL_CAN_ConfigFilter(&hcan2,&FILTER2);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}
void send_chassis_cur(int16_t a, int16_t b, int16_t c, int16_t d)
{ 
	uint8_t i[8];
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.StdId = 0x200;
  Tx1Message.DLC = 0x08;
	
	i[0]= a >> 8;i[1]= a;i[2]= b >> 8;i[3]= b;
	i[4]= c >> 8;i[5]= c;i[6]= d >> 8;i[7]= d;
	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,i,(uint32_t*)CAN_TX_MAILBOX0);
}

void send_fri_cur(int16_t a)//速度1000到1500
{ 
	uint8_t i[2];
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.DLC = 0x08;
	Tx1Message.StdId = 0x149;
	i[0]= a >> 8;i[1]= a;
	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,i,(uint32_t*)CAN_TX_MAILBOX0);
}

void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq)
{
	uint8_t Data[8];
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.StdId = 0x1ff;
  Tx1Message.DLC = 0x08;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  Data[0] = yaw_iq >> 8;
  Data[1] = yaw_iq;
  /* adding minus due to clockwise rotation of the gimbal motor with positive current */
  Data[2] = pit_iq >> 8;
  Data[3] = pit_iq;
  Data[4] = trigger_iq >> 8;
  Data[5] = trigger_iq;
  Data[6] = 0;
  Data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
void send_shoot_cur(int16_t iq1, int16_t iq2, int16_t iq3)
{
	uint8_t Data[8];
	Tx2Message.StdId   = 0x200;
  Tx2Message.IDE     = CAN_ID_STD;
  Tx2Message.RTR     = CAN_RTR_DATA;
  Tx2Message.DLC     = 0x08;
  Data[0] = iq1 >> 8;
  Data[1] = iq1;
  Data[2] = iq2 >> 8;
  Data[3] = iq2;
  Data[4] = iq3 >> 8;
  Data[5] = iq3;
	Data[6] = 0;
	Data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
void send_Gyro(uint8_t mode ,uint16_t time)//
{
	uint8_t Data[8];
	Tx2Message.StdId   = 100;
  Tx2Message.IDE     = CAN_ID_STD;
  Tx2Message.RTR     = CAN_RTR_DATA;
  Tx2Message.DLC     = 3;
	Data[0] = mode;
	Data[1] = time>>8;
	Data[2] = time;
	HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
void Gyro_Cali(uint16_t delay_time)
{
	//10ms持续发送，校准陀螺仪
	uint32_t tickstart = HAL_GetTick();
  uint32_t wait = delay_time;//ms

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(1u);
  }
  while((HAL_GetTick() - tickstart) < wait)
  {
		send_Gyro(0x30,800);//校准陀螺仪
  }
}
void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{ 
		//CAN1接收
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,DATA);
		switch (Rx1Message.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				i = Rx1Message.StdId - CAN_3508_M1_ID;
				encoder_data_handler3(&moto_chassis[i],DATA);
			}break;
			case CAN_YAW_MOTOR_ID:
			{
				encoder_data_handler4(&moto_yaw, DATA);
			}break;
			case CAN_PIT_MOTOR_ID:
			{
//			encoder_data_handler4(&moto_pit, DATA);
				#if (0)//不同小车，需要修改
				moto_pit.total_angle += 360;				
				#endif
			}break;
				case CAN_TRIGGER_MOTOR_ID:
			{
			encoder_data_handler3(&moto_trigger, DATA);
			}
			break;
			default:
			{
			}break;
		}	
	}
		
	else
	{
		//CAN2接收
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&Rx2Message,Data);
		switch (Rx2Message.StdId)
		{
			case 101:
			{
				zitai_data_receive2(&gyro_data,Data);//用哪个函数，看实际情况修改
			}
			break;
			default:
			{
			}break;
		}
	}
}

void encoder_data_handler(moto_measure_t* ptr, uint8_t Data[])
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(Data[0] << 8 | Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)//4096
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	 ptr->last_total_angle = ptr->total_angle;
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
  
#ifdef CHASSIS_3510
  int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
  ptr->speed_rpm   = (int16_t)(ptr->filter_rate * 7.324f);
#else
  ptr->speed_rpm     = (int16_t)(Data[2] << 8 | Data[3]);
  ptr->given_current = (int16_t)(Data[4] << 8 | Data[5]);
#endif

}

void encoder_data_handler3(moto_measure_t* ptr, uint8_t Data[])
{                
//        (ptr)->last_ecd = (ptr)->ecd;                                                          
//        (ptr)->ecd = (uint16_t)((Data[0] << 8 | Data[1]));       
					(ptr)->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]);     
//      	(ptr)->given_current = (uint16_t)(Data[4] << 8 | Data[5]);                                             
//				(ptr)->filter_rate = 	(ptr)->speed_rpm*0.10472;//W=2*PI*N
}
void encoder_data_handler4(moto_measure_t* ptr, uint8_t Data[])//6623
{                                                                                                                                    
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(Data[0] << 8 | Data[1]);
	if (ptr->ecd - ptr->last_ecd > 6500)
	{
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
	else if (ptr->ecd - ptr->last_ecd < -6500)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }
  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	ptr->last_total_angle = ptr->total_angle;
	ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
}

void zitai_data_receive2 (GYRO_DATA* gyro,uint8_t Data[])//
{
	//static int16_t pitch_connt = 0;
	static int16_t yaw_connt = 0;
	
	gyro->raw_v_x = Data[0]<<8 | Data[1];
	gyro->raw_v_z = Data[2]<<8 | Data[3];

	gyro->v_x = (float)gyro->raw_v_x * 0.057295f;
	gyro->v_z = (float)gyro->raw_v_z * 0.057295f;
	
	gyro->raw_pitch = Data[4]<<8 | Data[5];
	gyro->raw_yaw   = Data[6]<<8 | Data[7];
	
	gyro->pitch_angle = (float)gyro->raw_pitch/100;

	gyro->yaw_angle  = (float)gyro->raw_yaw/100;
	
	if(gyro->pitch_angle < 0)
	{
		gyro->pitch_angle = gyro->pitch_angle + 360;
	}
	
	if (gyro->pitch_angle>=299)
	{
	gyro->pitch=-(360-gyro->pitch_angle);
	}
	else
	{gyro->pitch=gyro->pitch_angle;}
	
	//gyro->pitch = gyro->pitch_angle + pitch_connt * 360;
	//gyro->last_pitch_angle = gyro->pitch_angle;
	
	if((gyro->yaw_angle - gyro->last_yaw_angle) > 330)
		yaw_connt--;
	else if((gyro->yaw_angle - gyro->last_yaw_angle) < -330)
		yaw_connt++;
	
	gyro->last_yaw = gyro->yaw;
	gyro->yaw = gyro->yaw_angle + yaw_connt * 360;
	gyro->last_yaw_angle = gyro->yaw_angle;
}
