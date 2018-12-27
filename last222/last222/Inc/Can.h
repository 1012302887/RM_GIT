#ifndef Can_H
#define Can_H
#include "main.h"
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
#define CHASSIS_3510 1
/* CAN send and receive ID */
typedef enum
{
	CAN_3510_M1_ID       = 0x201,
  CAN_3510_M2_ID       = 0x202,
	CAN_TRIGGER_MOTOR_ID = 0x207,
	
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
  CAN_YAW_MOTOR_ID     = 0x205,
  CAN_PIT_MOTOR_ID     = 0x206, 
  
  CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,

} can_msg_id_e;
/* can receive motor parameter structure */
#define FILTER_BUF 6
typedef struct
{
  uint16_t ecd;                            //编码器不经处理的原始值
  uint16_t last_ecd;                       //上一次的编码器不经处理的原始值
  
  int16_t  speed_rpm;                      //通过编码器计算得到的转速原始值
  int16_t  given_current;                  //通过编码器计算得到的电机电流原始值

  int32_t  round_cnt;                      //电机圈数
  int32_t  total_ecd;                      //经过处理后连续的编码器值
  float  total_angle;                      //电机角度(0~360度)
	float  last_total_angle;								 //电机上次角度(0~360度)
  
  uint16_t offset_ecd;										//初始编码器值
  uint32_t msg_cnt;                       //计数用
  
  int32_t  ecd_raw_rate;                  //通过编码器计算得到的角速度原始值
  int32_t  rate_buf[FILTER_BUF];          //buf，for filter
  uint8_t  buf_cut;                       //计算用
  int32_t  filter_rate;                   //角速度
	int32_t  last_filter_rate;              //上次角速度
} moto_measure_t;
typedef struct
{
	int16_t raw_pitch;           //未经处理的pit角度
	int16_t raw_roll;            //未经处理的roll角度
	int16_t raw_yaw;             //未经处理的yaw角度
	
	int16_t raw_v_x;             //未经处理的x轴角速度
	int16_t raw_v_y;             //未经处理的y轴角速度
	int16_t raw_v_z;             //未经处理的z轴角速度
	
	
	
	float pitch;                 //处理后的pit角度
	float roll;                  //处理后的roll角度
	float yaw;                   //处理后的yaw角度
	float	last_yaw;
	
	float pitch_angle;           //自己去看代码
	float last_pitch_angle;      //自己去看代码
	
	float yaw_angle;             //自己去看代码
	float last_yaw_angle;        //自己去看代码
	
	float v_x;                   //未经处理的x轴角速度
	float v_y;                   //未经处理的y轴角速度
	float v_z;                   //未经处理的z轴角速度
}	GYRO_DATA;
typedef struct
{
  uint16_t ecd;                            //编码器不经处理的原始值
  uint16_t last_ecd;                       //上一次的编码器不经处理的原始值
  
  int16_t  speed_rpm;                      //通过编码器计算得到的转速原始值
  int16_t  given_current;                  //通过编码器计算得到的电机电流原始值

  int32_t  round_cnt;                      //电机圈数
  int32_t  total_ecd;                      //经过处理后连续的编码器值
  float  total_angle;                      //电机角度
	float  last_total_angle;								 //电机上次角度
  
  uint16_t offset_ecd;										//初始编码器值
  uint32_t msg_cnt;                       //计数用
  
  int32_t  ecd_raw_rate;                  //通过编码器计算得到的角速度原始值
  int32_t  rate_buf[25];          	//buf，for filter
  uint8_t  buf_cut;                       //计算用
  int32_t  filter_rate;                   //角速度
	int32_t  last_filter_rate;              //上次角速度
} shoot_moto_measure_t;

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void send_chassis_cur(int16_t a, int16_t b, int16_t c, int16_t d);
void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq);
void send_shoot_cur(int16_t iq1, int16_t iq2, int16_t iq3);
void encoder_data_handler(moto_measure_t* ptr, uint8_t Data[]);
void encoder_data_handler1(moto_measure_t* ptr, uint8_t Data[]);
void encoder_data_handler2(shoot_moto_measure_t* ptr, uint8_t Data[]);
//void encoder_data_handler2(shoot_moto_measure_t* ptr, uint8_t Data[]);
void zitai_data_receive(GYRO_DATA* gyro,uint8_t Data[]);
void zitai_data_receive2 (GYRO_DATA* gyro,uint8_t Data[]);
void get_moto_offset(moto_measure_t* ptr, uint8_t Data[]);
void gyro_data_receive(GYRO_DATA* gyro,uint8_t Data[]);
void send_fri_cur(int16_t a);
extern moto_measure_t moto_chassis[];
//extern shoot_moto_measure_t moto_friction[];
extern moto_measure_t moto_yaw, moto_pit, moto_trigger;
extern shoot_moto_measure_t moto_friction[2];
extern moto_measure_t moto_trigger;
extern GYRO_DATA gyro_data;
#endif
