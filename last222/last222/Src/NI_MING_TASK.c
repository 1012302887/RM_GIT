#include "NI_MING.h"
void NI_MING_TASK(void const * argument)
{
		Ni_Ming(0xf1, chassis.wheel_spd_fdb[0],chassis.wheel_spd_fdb[1],chassis.wheel_spd_fdb[2],chassis.wheel_spd_fdb[3]);
		
//		Ni_Ming(0xf2, chassis.wheel_spd_ref[0],chassis.wheel_spd_ref[1],chassis.wheel_spd_ref[2],chassis.wheel_spd_ref[3]);
}
