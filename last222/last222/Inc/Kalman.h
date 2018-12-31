#ifndef KALMAN_H
#define KALMAN_H
#include "main.h"
//如果你需要滤波结果更依赖于观测量，那就调小R，增大Q；反之，调大R，调小Q，这样估计值就取决于系统。
//如果R大Q小，就是说，状态估计值比测量值要可靠，这时，所得出的结果就是更接近估计值；
//如果R小Q大，这时，计算出来的结果就会更接近测量值。
//A、H —— 系统参数，多输入多输出时为矩阵，单输入单输出时就是几个常数
/***************************************************************
	工程中用到的卡尔曼滤波是一个迭代的过程，每得到一个新的观测值迭代一次，
	来回来去地更新两个东西：“系统状态”(x)和“误差协方差”(P)。
	卡尔曼增益K(k)，和这一次的实际测量结果z(k)一起，
	用于修正系统状态x（预测）及误差协方差P（预测），得到最新的x(k|k)和P(k|k)：
****************************************************************/
typedef struct Kalman_t
{
	float _A_;
	float _H_;
	float _X_;//最新时刻系统的状态
	float _P_;//协方差
	float _K_;//可以不赋值
	//float _Z_;//最新观测值
	float _Q_;//过程噪声协方差
	float _R_;//测量噪声协方差
} Kalman_t;
void Kalman_filter_init(Kalman_t* Kalman_filter,float _P_,\
	float _Q_,float _R_);
float Kalman_filter_calc(Kalman_t* Kalman_filter,float Z);
extern Kalman_t zi_miao_kf[2];
extern Kalman_t CHASSIS_KF[4];
extern Kalman_t CHASSIS_REF_KF[4];
#endif
