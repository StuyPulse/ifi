/*******************************************************************************
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

//Speed constants

//Outputs
#define LEFT_DRIVE pwm02
#define RIGHT_DRIVE pwm01
#define ELEVATOR pwm05

//Digital sensor inputs
#define R_COUNTER rc_dig_in01
#define L_COUNTER rc_dig_in02

//Analog sensor inputs
#define GYRO rc_ana_in01

//Analog OI inputs
#define LEFT_Y p2_y
#define RIGHT_Y p1_y
#define AUTONOMOUS_CHOOSER p4_x


//Digital OI inputs

//#define ACQUISITOR_SPEED p4_y

/* Motor flips */
#define LEFT_FLIP 0
#define RIGHT_FLIP 0
#define SPIRAL_FLIP 0

#define REVERSE_PWM(x) (255 - (x)) 
 
 
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
 
 /* These routines reside in user_routines_fast.c */
 void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
 void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
 void Process_Data_From_Local_IO(void);
 
 
 
typedef struct
{

   unsigned char motor_R;
   unsigned char motor_L;
		
   float last_out_R;
   float last_out_L;

} robo_out;

typedef struct
{
  
  long rpm_1k_L;
  long rpm_1k_R;
	       
  float vel_L; //inches / sec
  float vel_R;
 
  int reverse_drive;
  int heading;
  
} robo_target;


typedef struct
{

  //Speeds
  long rpm_1k_L;
  long rpm_1k_R;

  float vel_L; //inches / sec
  float vel_R;
  
  //positions of individual parts
  float pos_L; //in inches
  float pos_R; 	
  
  //camera data
  int cam_tracking; //0 if target not seen, !0 if seen
  int cam_pan_angle; //in milliradians

  //position of robot
  int heading; //radians

  float x_pos;
  float y_pos; //position on field

} robo_state;

void control(robo_state * rs, robo_target * rt, robo_out * ro);
void guidance(robo_state * rs, robo_target * rt);
void output(robo_out * ro);
void update_state( robo_state * rs, robo_target *rt);
void robo_reset( robo_state * rs, robo_target * rt, robo_out *ro);

void Our_Autonomous_Code(void);
void Our_User_Code(void);
void Standard_Drive(robo_state *rs, robo_target *rt, robo_out *ro);

void LED_Control(void);
void LED_Reset(void);

int abs(int n);
 
#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
