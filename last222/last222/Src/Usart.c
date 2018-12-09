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
uint8_t PC_DATA[14];
RC_Ctl_t RC_CtrlData = {0};

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;
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
		HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
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
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);//¿ªÆô´®¿Ú¿ÕÏÐÖÐ¶Ï
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);	
}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)PC_DATA, RECEIVELEN);	
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
			//Ni_Ming(0xf1,RC_CtrlData.rc.ch0,RC_CtrlData.rc.ch1,RC_CtrlData.rc.ch2,RC_CtrlData.rc.ch3);
			//Ni_Ming(0xf2,RC_CtrlData.rc.s1,RC_CtrlData.rc.s2,ctrl_mode,0);
		}
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);
		SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, RECEIVELEN);
		__HAL_DMA_ENABLE(huart2.hdmarx);
	} 
}

void USART6_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart6.Instance->DR;
      tmp = huart6.Instance->SR;
      tmp--;
      //CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart6.hdmarx);
     	temp = huart6.hdmarx->Instance->NDTR;  
			if((RECEIVELEN - temp) == 7)
			{
				
			}
			
			 DMA2->LIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, RECEIVELEN);
      __HAL_DMA_ENABLE(huart6.hdmarx);
		}
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
}
