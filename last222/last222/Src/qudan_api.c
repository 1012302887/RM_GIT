#include "main.h"
void Up_Leg_To_Box_Command(void)
{
		chassis_leg.wheel_spd_ref[0] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[1] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[2] =  28*ramp_calc(&LegSpeedRamp);
		chassis_leg.wheel_spd_ref[3] =  28*ramp_calc(&LegSpeedRamp);
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
