#include "user_routines.h" //for definition (or not) of USE_NEW_OI

//modes for the arm (run by joystick or going to a certain position)
#define ARM_MANUAL 0
#define ARM_PICK_UP 1
#define ARM_HURDLE 2
#define ARM_POKE 3

#define ARM_CENTER_POS 1014

#define ARM_POKE_POS_HIGH 902
#define ARM_POKE_POS_LOW 862
#define ARM_HURDLE_POS 1080
#define ARM_PICKUP_POS 1210
#define ARM_STORE_POS 1236

#define ARM_POT 5 //rc_ana_in05
#define ARM pwm11

#ifdef USE_NEW_OI
#define ARM_JOY p3_y
#else
#define ARM_JOY p2_y
#endif

void init_arm(void);
void do_arm(void);
void do_puncher(void);
