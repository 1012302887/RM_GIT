#ifndef TOF_H_
#define TOF_H_
#include "main.h"
typedef struct TOF_MSG
{
	float Dist;
	float Strength;
	float Temp;
	uint16_t Checksum;
}TOF_MSG;
#endif