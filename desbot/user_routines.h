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
#define THE_ANSWER      42

/******These are the definitions of DESBOT'S OI******/
//Sets whether do_turn uses Ackermann or tank.
//#define USE_ACK
//Sets whether we're using the new OI or Tom's OI
//#define USE_NEW_OI

/* Constants */
#define OPEN 1
#define CLOSED 0

//arm inputs
#define ARM_OVERRIDE p1_sw_trig
#define ARM_SAVE_MODE AUTON1_SW

//Digital sensor inputs
//Encoders?
#define R_COUNTER rc_dig_in01
#define L_COUNTER rc_dig_in02

//Analog OI inputs
#ifdef USE_NEW_OI //defined (or not) above
#define X_DRIVE_AXIS p1_x
#define Y_DRIVE_AXIS p2_y

#define L_DRIVE_AXIS p2_y
#define R_DRIVE_AXIS p1_y
#else
#define X_DRIVE_AXIS p3_x
#define Y_DRIVE_AXIS p4_y

#define L_DRIVE_AXIS p4_y
#define R_DRIVE_AXIS p3_y
#endif

//center pot value for Ackermann
#define STEER_CENTER 556
//steering motor
#define STEER pwm12

 /*******************************************************************************
                            TYPEDEF DECLARATIONS
 *******************************************************************************/
//none.

 /*******************************************************************************
                           FUNCTION PROTOTYPES
   *******************************************************************************/
 /* These routines reside in user_routines.c */
 void User_Initialization(void);
 void Process_Data_From_Master_uP(void);
 void Default_Routine();
 
 /* These routines reside in user_routines_fast.c */
 void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
 void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
 void Process_Data_From_Local_IO(void);

 
typedef struct{
  unsigned char motor_R;
  unsigned char motor_L;

  unsigned char shift_servo;

  unsigned char grabber;
		
  float last_out_R;
  float last_out_L;
} robo_out;

typedef struct{
  long rpm_1k_L;
  long rpm_1k_R;
	       
  float vel_L; //inches / sec
  float vel_R;

  char grabber;
 
  int reverse_drive;
  int heading;
} robo_target;


typedef struct{
  //Speeds
  long rpm_1k_L;
  long rpm_1k_R;

  float vel_L; //inches / sec
  float vel_R;
  
  //positions of individual parts
  float pos_L; //in inches
  float pos_R; 	
  
  //position of robot
  int heading; //radians

  float x_pos;
  float y_pos; //position on field

  char grabber; //state of the grabber, UP or DOWN
} robo_state;

void control(robo_state * rs, robo_target * rt, robo_out * ro);
void guidance(robo_state * rs, robo_target * rt);
void output(robo_out * ro);
void update_state( robo_state * rs, robo_target *rt);
void robo_reset( robo_state * rs, robo_target * rt, robo_out *ro);

void Our_Autonomous_Code(void);
void Our_User_Code(void);
void Standard_Drive(robo_state *rs, robo_target *rt, robo_out *ro);
void Shift_Gear(robo_state *rs, robo_target *rt, robo_out *ro);
void Gear_Input(robo_state *rs, robo_target *rt);
int Lower_Grabber(robo_out *ro);
int Raise_Grabber(robo_out *ro);

void LED_Reset(void);

void Gyro_Drive(void);
void IR_Drive(void); 

#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
