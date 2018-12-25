/* second-order kalman filter on stm32 */
#include "main.h"
kalman_filter_init_t Kalman_filter_init={0} ;
kalman_filter_t Kalman_filter;
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
	mat_init(&F->z,2,1, (float *)I->z_data);
	mat_init(&F->A,2,2, (float *)I->A_data);
	mat_init(&F->H,2,2, (float *)I->H_data);
	mat_init(&F->AT,2,2,(float *)I->AT_data);
	mat_init(&F->Q,2,2, (float *)I->Q_data);// 预测过程噪声协方差
	mat_init(&F->R,2,2, (float *)I->R_data);// 测量过程噪声协方差
	mat_init(&F->P,2,2, (float *)I->P_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
	mat_trans(&F->A, &F->AT);
}

float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];

  return F->filtered_value;
}
void Kalman_Init(void)
{
	for(int i=0;i<4;i++)
	{
		Kalman_filter_init.A_data[i]=1;
	}
	for(int i=0;i<4;i++)
	{
		Kalman_filter_init.H_data[i]=1;
	}
	for(int i=0;i<4;i++)
	{
		Kalman_filter_init.Q_data[i]=0.01;
	}
	for(int i=0;i<4;i++)
	{
		Kalman_filter_init.R_data[i]=0.2;
	}
	kalman_filter_init(&Kalman_filter,&Kalman_filter_init);
}