#ifndef KALMAN_H
#define KALMAN_H
#include "main.h"
//�������Ҫ�˲�����������ڹ۲������Ǿ͵�СR������Q����֮������R����СQ����������ֵ��ȡ����ϵͳ��
//���R��QС������˵��״̬����ֵ�Ȳ���ֵҪ�ɿ�����ʱ�����ó��Ľ�����Ǹ��ӽ�����ֵ��
//���RСQ����ʱ����������Ľ���ͻ���ӽ�����ֵ��
//A��H ���� ϵͳ����������������ʱΪ���󣬵����뵥���ʱ���Ǽ�������
/***************************************************************
	�������õ��Ŀ������˲���һ�������Ĺ��̣�ÿ�õ�һ���µĹ۲�ֵ����һ�Σ�
	������ȥ�ظ���������������ϵͳ״̬��(x)�͡����Э���(P)��
	����������K(k)������һ�ε�ʵ�ʲ������z(k)һ��
	��������ϵͳ״̬x��Ԥ�⣩�����Э����P��Ԥ�⣩���õ����µ�x(k|k)��P(k|k)��
****************************************************************/
typedef struct Kalman_t
{
	float _A_;
	float _H_;
	float _X_;//����ʱ��ϵͳ��״̬
	float _P_;//Э����
	float _K_;//���Բ���ֵ
	//float _Z_;//���¹۲�ֵ
	float _Q_;//��������Э����
	float _R_;//��������Э����
} Kalman_t;
void Kalman_filter_init(Kalman_t* Kalman_filter,float _P_,\
	float _Q_,float _R_);
float Kalman_filter_calc(Kalman_t* Kalman_filter,float Z);
extern Kalman_t zi_miao_kf[2];
extern Kalman_t CHASSIS_KF[4];
extern Kalman_t CHASSIS_REF_KF[4];
#endif
