#include "main.h"
Kalman_t CHASSIS_KF[4];
Kalman_t zi_miao_kf[2];
Kalman_t GIMBAL_KF[4];
void Kalman_filter_init(Kalman_t* Kalman_filter,float _P_,\
	float _Q_,float _R_)
{
	Kalman_filter->_A_=1;
	Kalman_filter->_H_=1;
	Kalman_filter->_P_=_P_;
	Kalman_filter->_Q_=_Q_;
	Kalman_filter->_R_=_R_;
}

float Kalman_filter_calc(Kalman_t* Kalman_filter,float Z)
{
	//公式一：K(k) = P(k|k-1) / (P(k|k-1) + R)
	Kalman_filter->_K_ = Kalman_filter->_P_/(Kalman_filter->_P_+Kalman_filter->_R_);
	//公式二：x(k|k) = x(k-1|k-1) + K(k) · (z(k) - x(k-1|k-1))
	Kalman_filter->_X_ = Kalman_filter->_X_+ (Kalman_filter->_K_*(Z-Kalman_filter->_X_));
	//公式三：P(k+1|k) = (1 - K(k)) · P(k|k-1) + Q
	Kalman_filter->_P_ = (1-Kalman_filter->_K_)*Kalman_filter->_P_+Kalman_filter->_Q_;
	return Kalman_filter->_X_;
}
