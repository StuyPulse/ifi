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

/* Shortcut for logical (x-5 <= in <= x+5) */
#define ISNEAR(in,x) ((int)in >= (x-5) && (int)in <= (x+5))

/* Constatnts */
#define RAMP_SERVO_UNDEPLOYED 0
#define RAMP_SERVO_DEPLOYED 255
#define DEPLOYED 1
#define UNDEPLOYED 0
#define GYRO_ENABLED 1

#define RIGHT_JOY p3
#define LEFT_JOY p4

#define HIGH_SERVO_VALUE 255
#define LOW_SERVO_VALUE 0
#define MIDDLE_SERVO_VALUE 127

#define POS1 370
#define POS2 544
#define POS3 714 
#define POS4 941
#define ARM_SCORE_INC 20
#define POS5_GET_RING 350
#define ARM_TOP POS1
#define ARM_BOTTOM POS4

//arm inputs
#define ARM_JOY p1_y
#define ARM_OVERRIDE p1_sw_trig

//y joysticks
#define JOY_LEFT_Y leftjoyy
#define JOY_RIGHT_Y rightjoyy

//buttons
#define LEFT_RAMP_DEPLOY p2_sw_top
#define RIGHT_RAMP_DEPLOY p2_sw_aux2
#define LEFT_RAMP_RAISE p2_sw_trig

#define LANCE_RAISE p1_sw_trig
#define LANCE_LOWER p1_sw_top

#define POS1_SW ISNEAR(p2_x,250)
#define POS2_SW ISNEAR(p2_x,224)
#define POS3_SW ISNEAR(p2_x,198)
#define POS4_SW ISNEAR(p2_x,172)
#define ARM_SCORE_SW ISNEAR(p2_x, 148)
//#define POS5_GET_RING_SW ISNEAR(p2_x, 123)
#define AUTON1_SW ISNEAR(p2_y,250)
#define AUTON2_SW ISNEAR(p2_y,223)
#define AUTON3_SW ISNEAR(p2_y,198)
#define AUTON4_SW ISNEAR(p2_y,172)
#define AUTON5_SW ISNEAR(p2_y,148)
#define AUTON6_SW ISNEAR(p2_y,123)

#define FOUR_MOTOR

//#define TWO_MOTOR

//Speed constants (PWM outputs) 

//Motor Outputs
#ifdef TWO_MOTOR
#define LEFT_DRIVE pwm01
#define RIGHT_DRIVE pwm02
#endif
#ifdef FOUR_MOTOR
#define LEFT1_DRIVE pwm02
#define LEFT2_DRIVE pwm04
#define RIGHT1_DRIVE pwm01
#define RIGHT2_DRIVE pwm03
#endif

#define SHIFT pwm05

//The motor to rotate the arm
#define ARM_MOTOR pwm06

//The motors that raise the ramps
#define L_RAMP_MOTOR pwm09
#define R_RAMP_MOTOR pwm10

//The servos that deploy the ramps
#define L_RAMP_SERVO pwm07
#define R_RAMP_SERVO pwm08

#define LANCE_SERVO pwm11

//Digital sensor inputs
//Encoders?
#define R_COUNTER rc_dig_in01
#define L_COUNTER rc_dig_in02

//Analog sensor inputs
#define GYRO rc_ana_in01
#define ARM_POT rc_ana_in02

#define LF_CURRENT rc_ana_in03
#define LB_CURRENT rc_ana_in04
#define RF_CURRENT rc_ana_in05
#define RB_CURRENT rc_ana_in06



//Analog OI inputs
#define LEFT_Y p4_y
#define RIGHT_Y p3_y

//Digital OI inputs
#define REVERSE_SW p4_sw_trig
#define FORWARD_SW p3_sw_trig
#define HIGH_SW (p3_sw_aux1)
#define LOW_SW (p4_sw_top)
#define HIGH_GEAR_TOGGLE (!p2_sw_aux1)

/* Motor flips */
#define LEFT_FLIP 0
#define RIGHT_FLIP 1

#define REVERSE_PWM(x) (255 - (x)) 
#define INRANGE(x,y,r) ((y)-(x) < (r) && (y)-(x) > -(r) ? 1 : 0)
 
#define ALLOWABLE_PAN_ERROR 5

#define ARM_P_GAIN_N 7
#define ARM_P_GAIN_D 4
#define ARM_I_GAIN_N 0
#define ARM_I_GAIN_D 100

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

#define MIDDLE 2
#define HIGH 1
#define LOW 0



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
  unsigned char lance_servo;

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
  char gear;
  char lance;
 
  int reverse_drive;
  int heading;

  int right_ramp;
  int left_ramp;
  
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

  char gear; //current gear... in the gear box...
  char lance; //state of the lance, UP or DOWN

  int right_ramp;
  int left_ramp;

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
