#include "main.h"
uint8_t Up_Leg_To_Box_Command(void)
{
		chassis_leg.wheel_spd_ref[0] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[1] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[2] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[3] =  28*ramp_calc(&LegSpeedRamp);
	if(moto_chassis[0].total_angle>=34500||moto_chassis[1].total_angle>=34500||moto_chassis[2].total_angle>=34500||\
			moto_chassis[3].total_angle>=34500)
		{
			chassis_leg.wheel_spd_ref[0] =  0;
			chassis_leg.wheel_spd_ref[1] =  0;
			chassis_leg.wheel_spd_ref[2] =  0;
			chassis_leg.wheel_spd_ref[3] =  0;
			leg_mode = leg_normal_mode;
			ramp_init(&LegSpeedRamp,1500);
			return 0;
		}
		return 1;
}
void Push_command()
{
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
}
void Catch_command()
{
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1);
}
void Bounce_command()
{
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);
}	
