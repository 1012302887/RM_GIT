#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
#define W_KEY 			0x0001		
#define S_KEY 			0x0002
#define A_KEY  			0x0004
#define D_KEY  			0x0008
#define SHIFT_KEY  	0x0010
#define CTRL_KEY  		0x0020 //¾É°æÎªCTRL
#define Q_KEY  			0x0040
#define E_KEY 				0x0080
#define R_KEY  			0x0100
#define F_KEY  			0x0200
#define G_KEY  			0x0400
#define Z_KEY  			0x0800
#define X_KEY  			0x1000
#define C_KEY  			0x2000
#define V_KEY  			0x4000		
#define B_KEY 				0x8000
/******************************************************/
#endif
