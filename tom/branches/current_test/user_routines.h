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
 /* Macros are substituted in at compile time and make your code more readable */
 /* as well as making it easy to change a constant value in one place, rather  */
 /* than at every place it is used in your code.           
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


/******These are the definitions of TOM'S OI******/
/* Constatnts */
#define RAMP_SERVO_UNDEPLOYED 127
#define RAMP_SERVO_DEPLOYED 255

//arm inputs
#define ARM_JOY p3_y
#define ARM_OVERRIDE p3_sw_trig

//y joysticks
#define JOY_LEFT_Y leftjoyy
#define JOY_RIGHT_Y rightjoyy

//buttons
#define LEFT_RAMP_DEPLOY p4_sw_trig
#define RIGHT_RAMP_DEPLOY p4_sw_top
#define RAISE_RAMP_L_BUTTON leftrampbutton
#define RAISE_RAMP_R_BUTTON rightrampbutton


#define FOUR_MOTOR
//#define TWO_MOTOR

//Speed constants (PWM outputs) 

//Motor Outputs
#ifdef TWO_MOTOR
#define LEFT_DRIVE pwm01
#define RIGHT_DRIVE pwm02
#endif
#ifdef FOUR_MOTOR
#define LEFT1_DRIVE pwm01
#define LEFT2_DRIVE pwm02
#define RIGHT1_DRIVE pwm03
#define RIGHT2_DRIVE pwm04
#endif

#define SHIFT pwm05

//The motor to rotate the arm
#define ARM_MOTOR pwm06

//The motors that raise the ramps
#define L_RAMP_MOTOR pwm07
#define R_RAMP_MOTOR pwm08

//The servos that lower the ramps
#define L_RAMP_SERVO pwm09
#define R_RAMP_SERVO pwm10

//Digital sensor inputs
//Encoders?
#define R_COUNTER rc_dig_in01
#define L_COUNTER rc_dig_in02

//Analog sensor inputs
#define GYRO rc_ana_in01
#define ARM_POT rc_ana_in02

#define LF_CURRENT rc_ana_in04
#define LB_CURRENT rc_ana_in06
#define RF_CURRENT rc_ana_in03
#define RB_CURRENT rc_ana_in05



//Analog OI inputs
#define LEFT_Y p2_y
#define RIGHT_Y p1_y

//Digital OI inputs
#define REVERSE_INPUT (p1_sw_trig && p2_sw_trig)
#define GEAR_SHIFT (p1_sw_top)

/* Motor flips */
#define LEFT_FLIP 0
#define RIGHT_FLIP 1

#define REVERSE_PWM(x) (255 - (x)) 
#define INRANGE(x,y,r) ((y)-(x) < (r) && (y)-(x) > -(r) ? 1 : 0)
 
#define ALLOWABLE_PAN_ERROR 5

#define ARM_P_GAIN_N 3
#define ARM_P_GAIN_D 2
#define ARM_I_GAIN_N 1
#define ARM_I_GAIN_D 50

#define P_GAIN_R_N 1
#define P_GAIN_R_D 33
#define I_GAIN_R_N 0
#define I_GAIN_R_D 4
#define D_GAIN_R_N 1
#define D_GAIN_R_D 4

#define P_GAIN_L_N 1
#define P_GAIN_L_D 33
#define I_GAIN_L_N 0
#define I_GAIN_L_D 4
#define D_GAIN_L_N 1
#define D_GAIN_L_D 4

#define OFF 0
#define ON 1

#define ARM_TOP 200
#define ARM_BOTTOM 893

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

  unsigned char ramp_motor_l;
  unsigned char ramp_motor_r;
  unsigned char ramp_servo_l;
  unsigned char ramp_servo_r;
  
  unsigned char shift_servo;

  unsigned char arm;
		
  float last_out_R;
  float last_out_L;
  
} robo_out;

typedef struct
{
  
  long rpm_1k_L;
  long rpm_1k_R;
	       
  float vel_L; //inches / sec
  float vel_R;

  int arm_pos;
 
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

  int left_front_I; //mA
  int left_back_I;
  int right_front_I;
  int right_back_I;  

  unsigned char gear; //current gear... in the gear box...

  int arm_pos;

} robo_state;

void control(robo_state * rs, robo_target * rt, robo_out * ro);
void guidance(robo_state * rs, robo_target * rt);
void output(robo_out * ro);
void update_state( robo_state * rs, robo_target *rt);
void robo_reset( robo_state * rs, robo_target * rt, robo_out *ro);

void Our_Autonomous_Code(void);
void Our_User_Code(void);
void Standard_Drive(robo_state *rs, robo_target *rt, robo_out *ro);
void Set_Arm_Pos(robo_state *rs, robo_target *rt, robo_out *ro);

void turn_to_camera(robo_out *ro);
void LED_Reset(void);

int abs(int n);
 
#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/