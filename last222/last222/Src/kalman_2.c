#include "main.h"
float K,P,Q,R,X1;
float Kalman_simple(float Z)
{
	K= P/(P+R);	
	X1=X1+K*(Z-X1);
	P = (1-K)*P+Q;
	return X1;
}
void Kalman_simple_init(float p,float q,float r,float x1)
{
P=p;Q=q;R=r;X1=x1;
}
