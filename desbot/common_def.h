#ifndef _comdef
#define _comdef

#include "user_routines.h" //for definition (or not) of USE_NEW_OI

#define RIGHT_SIDE	1
#define LEFT_SIDE	2

#define PI		3.14159265358979323846264338327950288419716939937510582097494459230781640
#define DRIVE_WHEEL_DIA	6.0 //inches
#define WHEEL_CIRC	(PI * DRIVE_WHEEL_DIA)
#define WHEEL_BASE	(22.75)
#define NUM_DRIVE_SPOKE	6

/*********** Outputs ***********/
#define L_DRIVE1 pwm02
#define L_DRIVE2 pwm04
#define R_DRIVE1 pwm01
#define R_DRIVE2 pwm03

//compressor
#define PUMP_F relay4_fwd
#define PUMP_R relay4_rev
//pressure switch
#define P_SWITCH rc_dig_in03
//big arm tilt pneumatic
#define TILT_F relay5_fwd
#define TILT_R relay5_rev
//puncher
#define PUNCH_F relay7_fwd
#define PUNCH_R relay7_rev
//grabber
#define GRAB_F relay6_fwd
#define GRAB_R relay6_rev
//limit switch on puncher pneumatic
#define PUNCH_SWITCH rc_dig_in14
//firing pin
#define FIREPIN_F relay8_fwd
#define FIREPIN_R relay8_rev

//#define VAC relay7_fwd //not there anymore, eh

//the switch reads as 1 when off and 0 when on
#define AUTON_ON (!rc_dig_in13)

#define CORNER_SW_UP (rc_dig_in11 && !rc_dig_in12)
#define CORNER_SW_MID (rc_dig_in11 && rc_dig_in12)
#define CORNER_SW_DOWN (!rc_dig_in11 && rc_dig_in12)

#ifdef USE_NEW_OI
#define GRAB_BUTTON (p4_sw_aux2 || p3_sw_trig) //actually the top button!
#define PUNCH_BUTTON p3_sw_top //actually the trigger!
#else
#define GRAB_BUTTON p2_sw_top
#define PUNCH_BUTTON p2_sw_trig
#endif



/* Shortcut for logical (x-5 <= in <= x+5) */
#define REVERSE_PWM(x) (255 - (x)) 
#define INRANGE(x,y,r) ((y)-(x) < (r) && (y)-(x) > -(r))
#define ISNEAR(in,x) ((int)in >= (x-5) && (int)in <= (x+5))

#endif
