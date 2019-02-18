#ifndef __first_order_H__
#define __first_order_H__
#include "main.h"
typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num;       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
extern first_order_filter_type_t chassis_fdb_first[4];
#endif
