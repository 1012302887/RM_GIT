#ifndef _CAR_NUM_H_
#define _CAR_NUM_H_

#define       CAR_NUM      4
#define				HATCH_COVER    TIM4->CCR1

#if   CAR_NUM == 1
	#define 			OPEN_COVER		 			1380
	#define  			CLOSE_COVER    			2350
  #define				AUTOSHOOT_X_OFFSET  0
#elif CAR_NUM == 4
	#define 			OPEN_COVER		      888
	#define  			CLOSE_COVER         1766
  #define				AUTOSHOOT_X_OFFSET  -2
	#define       MOTO_YAW_OFFSET_ECD  7600
	#define       MOTO_PIT_OFFSET_ECD  1800
#else
	#define 			OPEN_COVER		      1300
	#define  			CLOSE_COVER         2150
  #define				AUTOSHOOT_X_OFFSET  0
#endif
#endif
