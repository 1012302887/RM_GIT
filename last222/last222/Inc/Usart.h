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
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;
typedef struct
{
	int16_t raw_yaw_angle;
	int16_t raw_pit_angle;
	
	float filter_yaw;
	float	filter_pit;
	
  float dynamic_yaw;
	float dynamic_pit;
	
	uint16_t star_shoot;
	uint16_t last_star_shoot;
	uint16_t bug;
	uint16_t last_bug;
	
	uint16_t time;
	
	float last_dynamic_yaw;
	float last_dynamic_pit;
	
	float coordinate;
	float coordinate_data[5];
	float last_coordinate;
	float last_coordinate_data[5];
	
	float last_times;
	float now_times;
	
	float v_now;
	float v_last;
	float v_now_i;
	
	float	yaw_befoer[101];
	float	pit_befoer[101];
	uint8_t shoot_data;
} pc_data_t;
extern pc_data_t pc_data;
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
void Get_Remote_info(RC_Ctl_t *rc, uint8_t *pData);
void USART6_Transmit(void);
extern uint8_t SEND_DATA[7];//´®¿ÚÁù·¢ËÍ
extern uint8_t Rx_data[8];
#endif
