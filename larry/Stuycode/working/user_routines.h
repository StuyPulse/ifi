\/*******************************************************************************
 * FILE NAME: user_routines.h
 *
 * DESCRIPTION: 
 *  This is the include file which corresponds to user_routines.c and
 *  user_routines_fast.c
 *  It contains some aliases and function prototypes used in those files.
 *
 * USAGE:
 *  If you add your own routines to those files, this is a good place to add
 *  your custom macros (aliases), type definitions, and function prototypes.
 *******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


 /*******************************************************************************
                            MACRO DECLARATIONS
 *******************************************************************************/
 /* Add your macros (aliases and constants) here.                              */
 /* Do not edit the ones in ifi_aliases.h                                      */
 /* Macros are substituted in at compile time and make your code more readable */
 /* as well as making it easy to change a constant value in one place, rather  */
 /* than at every place it is used in your code.                               */
 /*
   EXAMPLE CONSTANTS:
   #define MAXIMUM_LOOPS   5
   #define THE_ANSWER      42
   #define TRUE            1
   #define FALSE           0
   #define PI_VAL          3.1415
   
   EXAMPLE ALIASES:
   #define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
   #define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
 */
 
 /* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */
 
/*Yvette's aliases*/

#define OFF 0
#define FORWARD 1
#define REVERSE 2


#define JOY_DEAD_ZONE 20

//Speed constants

//Victor Outputs
#define LEFT_DRIVE pwm02
#define RIGHT_DRIVE pwm01
#define UPPER_ARM pwm05

//Spike Outputs
#define BALL_GRABBER relay1_fwd
#define GOAL_GRABBER relay1_rev
#define LOWER_ARM_FWD relay2_fwd
#define LOWER_ARM_REV relay2_rev
#define LEFT_WING relay3_fwd
#define RIGHT_WING relay3_rev
#define PUMP relay8_fwd

//Digital sensor inputs
#define PUMP_IN (Get_Analog_Value(rc_ana_in01)<=8)

//Analog sensor inputs

//Analog OI inputs
#define LEFT_IN p1_wheel
#define RIGHT_IN p1_y
#define UPPER_ARM_IN p2_y

//Digital OI inputs
#define LEFT_WING_IN (!p4_sw_trig)
#define RIGHT_WING_IN (!p4_sw_top)
#define GOAL_GRABBER_IN p4_sw_aux1
#define LOWER_ARM_UP p3_sw_aux1
#define LOWER_ARM_DOWN p3_sw_aux2
#define BALL_GRABBER_OPEN p3_sw_top
#define BALL_GRABBER_CLOSE p3_sw_trig

//#define GORDON_OVERRIDE p4_sw_aux2

/* Motor flips */
#define LEFT_FLIP 0
#define RIGHT_FLIP 0
#define UPPER_ARM_FLIP 1

#define REVERSE_PWM(x) (x = 255 - (x)) 
 
 
 /*******************************************************************************
                            TYPEDEF DECLARATIONS
 *******************************************************************************/
 /* EXAMPLE DATA STRUCTURE */
 /*
   typedef struct
   {
   unsigned int  NEW_CAPTURE_DATA:1;
   unsigned int  LAST_IN1:1;
   unsigned int  LAST_IN2:1;
   unsigned int  WHEEL_COUNTER_UP:1;
   unsigned int  :4;
   unsigned int wheel_left_counter;
   unsigned int wheel_right_counter;
   } user_struct;
 */
 
 /*******************************************************************************
                           FUNCTION PROTOTYPES
   *******************************************************************************/
 
 /* These routines reside in user_routines.c */
 void User_Initialization(void);
 void Process_Data_From_Master_uP(void);
 void Default_Routine(void);

 
 /* These routines reside in user_routines_fast.c */
 void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
 void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
 void Process_Data_From_Local_IO(void);
 
 
#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
