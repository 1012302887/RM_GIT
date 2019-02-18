#ifndef __first_order_H__
#define __first_order_H__
#include "main.h"
typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num;       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
extern first_order_filter_type_t chassis_fdb_first[4];
#endif
