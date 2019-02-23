#ifndef Usart_H
#define Usart_H
#include "main.h"

#define RECEIVELEN 128
/*remote info*/
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
	int16_t bogan;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;
typedef struct TOF_MSG
{
	float Dist;
	float Strength;
	float Temp;
	uint16_t Checksum;
}TOF_MSG;
extern TOF_MSG TOF_DATA;
extern uint8_t Send_Pc_Data[4]; 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern RC_Ctl_t RC_CtrlData ;
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_USART3_UART_Init(void);
void Get_Remote_info(RC_Ctl_t *rc, uint8_t *pData);
void USART6_Transmit(void);
extern uint8_t SEND_DATA[7];//´®¿ÚÁù·¢ËÍ
extern uint8_t Rx_data[8];
#endif
