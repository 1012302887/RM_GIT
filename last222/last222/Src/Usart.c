#include "main.h"
#include "math.h"

#if 1
#pragma import(__use_no_semihosting)                     
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
void _sys_exit(int x) 
{ 
	x = x; 
} 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);
	USART1->DR = (uint8_t) ch;      
	return ch;
}
#endif 

uint8_t rc_RxBuffer[RECEIVELEN];
uint8_t Rx_data[8];
RC_Ctl_t RC_CtrlData = {0};
pc_data_t pc_data = {0};//
uint8_t Send_Pc_Data[4]; 
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart3_rx;
/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//		HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);	
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);//�������ڿ����ж�.
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);	
}
/* USART3 ����ϵͳ */
void MX_USART3_UART_Init(void)
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart3);
	
	SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)JudgeDataBuffer, 1024);
}

uint16_t	temp;
void USART2_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) && 
		__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
	{
		uint16_t tmp = huart2.Instance->DR;
		tmp = huart2.Instance->SR;
		tmp--;
		CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
		__HAL_DMA_DISABLE(huart2.hdmarx);

		temp = huart2.hdmarx->Instance->NDTR; 
		if((RECEIVELEN - temp) == 18)
		{
			Get_Remote_info(&RC_CtrlData ,rc_RxBuffer);
		}
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);
		SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, RECEIVELEN);
		__HAL_DMA_ENABLE(huart2.hdmarx);
	} 
}
void USART3_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE))
    {
      uint16_t tmp = huart3.Instance->DR;
      tmp = huart3.Instance->SR;
      tmp--;
      //CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart3.hdmarx);
			
     	temp = huart3.hdmarx->Instance->NDTR;  
			if((1024 - temp) >=1 && (1024 - temp) <=222)
				Judge_Receive(JudgeDataBuffer);
			}
      DMA1->LIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
      __HAL_DMA_SET_COUNTER(huart3.hdmarx, 1024);
      __HAL_DMA_ENABLE(huart3.hdmarx);
} 

void Get_Remote_info(RC_Ctl_t *rc, uint8_t *pData)
{
	Reset ++;
	rc->rc.ch0 = (pData[0] | pData[1] << 8) & 0x07FF;
  rc->rc.ch0 -= 1024;
  rc->rc.ch1 = (pData[1] >> 3 | pData[2] << 5) & 0x07FF;
  rc->rc.ch1 -= 1024;
  rc->rc.ch2 = (pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
  rc->rc.ch2 -= 1024;
  rc->rc.ch3 = (pData[4] >> 1 | pData[5] << 7) & 0x07FF;
  rc->rc.ch3 -= 1024;
 
	/* prevent remote control zero deviation */
  if(rc->rc.ch0 <= 10 && rc->rc.ch0 >= -10)
    rc->rc.ch0 = 0;
  if(rc->rc.ch1 <= 10 && rc->rc.ch1 >= -10)
    rc->rc.ch1 = 0;
  if(rc->rc.ch2 <= 10 && rc->rc.ch2 >= -10)
    rc->rc.ch2 = 0;
  if(rc->rc.ch3 <= 10 && rc->rc.ch3 >= -10)
    rc->rc.ch3 = 0;
	
 rc->rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 rc->rc.s2 = ((pData[5] >> 4) & 0x0003);

 rc->mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 rc->mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 rc->mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

 rc->mouse.press_l = pData[12];
 rc->mouse.press_r = pData[13];

 rc->key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

 rc->key.bogan=((int16_t)pData[16]) | ((int16_t)pData[17] << 8);
 rc->key.bogan-=1024;
}
