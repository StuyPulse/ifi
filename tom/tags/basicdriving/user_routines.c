/*******************************************************************************
 * FILE NAME: user_routines.c <FRC VERSION>
 *
 * DESCRIPTION:
 *  This file contains the default mappings of inputs  
 *  (like switches, joysticks, and buttons) to outputs on the RC.  
 *
 * USAGE:
 *  You can either modify this file to fit your needs, or remove it from your 
 *  project and replace it with a modified copy. 
 *
 *
 * Note 1/5/07 I pulled out all the non-year-independent code, and really, 
 * anything excess.
 *******************************************************************************/

#include <stdio.h>
#include <math.h>

/* Headers from default code */
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"

/* Headers from Kevin Watson */
//#include "adc.h"
#include "camera.h"
#include "serial_ports.h"
#include "tracking.h"
#include "gyro.h"
#include "interrupts.h"

/* Headers by us */
#include "velocity.h"
#include "controller.h"
#include "common_def.h"
#include "trajectory.h"


extern volatile trajectory traj_array[];

/* These are variables that control our PID controllers */
extern volatile unsigned int left_clock;
extern volatile unsigned int right_clock;
extern volatile int left_vel;
extern volatile int right_vel;
extern volatile unsigned int left_index;
extern volatile unsigned int right_index;


/* Structures for the robot guidance and controller systems */
robo_state r_state;
robo_target r_target;
robo_out r_out;

/* Shortcut for logical (x-5 <= in <= x+5) */
#define ISNEAR(in,x) ((int)in >= (x-5) && (int)in <= (x+5) )

/* Preprocessor definitions to control different 'modes' of code */
#define OUR_DEBUG



/*******************************************************************************
 * FUNCTION NAME: User_Initialization
 * PURPOSE:       This routine is called first (and only once) in the Main function.  
 *                You may modify and add to this function.
 * CALLED FROM:   main.c
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void User_Initialization (void)
{
  rom const char *strptr = "Gyro + Encoders\r";

  //  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

  /* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
  /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
     is the same as the following:

     digital_io_01 = INPUT;
     digital_io_02 = INPUT;
     ...
     digital_io_04 = INPUT;
  */

  /* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

  /* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

  /* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

  /* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
     /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

     CCP2CON = 0x3C;
     PR2 = 0xF9;
     CCPR2L = 0x7F;
     T2CON = 0;
     T2CONbits.TMR2ON = 1;

     Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  //Initialize_Serial_Comms();
#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

  /* Begin stuff for sensors and other systems */
  Init_Serial_Port_One();
  Init_Serial_Port_Two();
  Initialize_PWM();
  // Initialize interrupts and timers
  //Initialize_ADC();
  Initialize_Gyro();

  Initialize_Interrupts();
  //Initialize_Timer_0();     DO NOT USE TIMER0 - IT PROVIDES SOME BENEFIT THAT WE SHOULD AVOID LOSING.
  Initialize_Timer_1();
  //  Initialize_Timer_2(); //Using ADC timer2 instead.
  //  Initialize_Timer_3(); Do not use this, Kevin's PWM code does
  Initialize_Timer_4();   
  Polled_Mode(); // Pause the camera before we send our command

  //PAN_SERVO = PAN_CENTER_PWM_DEFAULT;
  //TILT_SERVO = TILT_MIN_PWM_DEFAULT;
  /* End stuff for sensors and other systems */

  /* Begin stuff totally by us */
  robo_reset( &r_state, &r_target, &r_out);
  init_traj();
  /* End stuff totally by us */

  Putdata(&txdata);             /* DO NOT CHANGE! */

  printf("%s2007 Robot -- Tom -- Ready to roll! \n", strptr);       /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/* In case you're wondering, the u in uP is meant to be a mu, which is the 
 *  metric prefix for micro (one millionth).  uP means microprocessor! */
/*******************************************************************************
 * FUNCTION NAME: Process_Data_From_Master_uP
 * PURPOSE:       Executes every 26.2ms when it gets new data from the master 
 *                microprocessor.
 * CALLED FROM:   main.c
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void Process_Data_From_Master_uP(void){

  /* This calibrates the gyro as soon as the robot turns on.
   *  It takes about 11 seconds, during which the robot is disabled.  */
  static unsigned int j=0;
  if(j<=300)  j++  ; // Stop after gyro is calibrated
  
  if(j == 10)
    {
      printf("\rCalculating Gyro Bias...");
    }
  
  if(j == 60)
    {
      // start a gyro bias calculation
      Start_Gyro_Bias_Calc();
    }

  if(j == 300)
    {
      // terminate the gyro bias calculation
      Stop_Gyro_Bias_Calc();

      // reset the gyro heading angle
      Reset_Gyro_Angle();

      printf("Done\r");
    }

  /* This gets PWM values */
  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
 
  /* Once we're done calibrating, we start controlling the robot. */
  if ( j >= 300 )
    {
      //printf("%d, %d\r",Get_Gyro_Rate(),(int)Get_Gyro_Angle());
      Our_User_Code();      
      Tracking_Info_Terminal();
    }



  /* These PWMs function a little differently than the others.  Avoid using them. */
  PWM(pwm13,pwm14,pwm15,pwm16);

  /* This sends out the PWMs */
  Putdata(&txdata);             /* DO NOT CHANGE! */
}


/*******************************************************************************"****************
 ***********************************     OUR STUFF  *********************************************
 ***********************************************************************************************/


/*******************************************************************************
 * FUNCTION NAME: Our_Autonomous_Code
 * PURPOSE:       The code we execute during autonomous mode.
 * CALLED FROM:   Process_Data_From_Master_uP
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void Our_Autonomous_Code()
{

  Camera_Handler();
  Servo_Track();
  update_state(&r_state,&r_target);

  /* We use our guidance system to figure out what physical state we want the bot */
  guidance(&r_state, &r_target);
  /* Then, the controller figures out what needs to put in our outputs to meet out 
   *  target state */
  control(&r_state, &r_target, &r_out);
  /* And finally sets the outputs to the values we decide they should be at. */
  output(&r_out);
}

/*******************************************************************************
 * FUNCTION NAME: Our_User_Code
 * PURPOSE:       The code we execute during user-controlled (as well as disabled) mode.
 * CALLED FROM:   Process_Data_From_Master_uP
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void Our_User_Code()
{

  Camera_Handler();
  //split_window(1);
  Servo_Track();  

  update_state(&r_state,&r_target);
 
  /* Standard drive runs whatever code we want during user mode.
   *  Because parts of it may be closed-loop, it is in the same framework
   *  we use for autonomous. */

  Standard_Drive(&r_state, &r_target, &r_out);
  if(p1_sw_trig)
    turn_to_camera(&r_out);
  /* Finally, we send the data out to the motors and other outputs. */
  output(&r_out);  

  if(user_display_mode)
    {
      User_Mode_byte = backup_voltage*10;
    }
  /***************TEMPORARY **********************/
  //ARM_MOTOR = p3_y;
  
  if(user_display_mode)
    {
      User_Mode_byte = Get_Analog_Value(ARM_POT) / 4;
    }
  else
    printf("arm pot: %d\r\n",Get_Analog_Value(ARM_POT));
  Pwm1_red = rc_dig_in01;
  Pwm2_red = rc_dig_in02;
  
  
}



/*******************************************************************************
 * FUNCTION NAME: Standard_Drive
 * PURPOSE:       Maps joystick inputs to motors
 * CALLED FROM:   Our_User_Code
 * ARGUMENTS:     robo_state, robo_target, robo_out
 * RETURNS:       void
 *******************************************************************************/
void Standard_Drive(robo_state *rs, robo_target *rt, robo_out *ro)
{
  int tmp;
  static char already_shifted=0;

#ifdef USING_CURRENT_SENSORS
  /* This all should maybe go into a function */
  static int controlling=0;
  static struct pidstate left,right;
  if(p2_sw_trig) //replace this with a #define when I'm feeling less lazy
    {
      if(controlling==0)
	{
	  init_controller_divisor(&left,1,10,0,0,127,-127);
	  init_controller_divisor(&right,1,10,0,0,127,-127);
	  controlling=1;
	}
      ro->motor_L = controller(&left,LEFT_IDEAL_CURRENT,rs->left_front_I+rs->left_back_I);
      ro->motor_R = controller(&left,RIGHT_IDEAL_CURRENT,rs->right_front_I+rs->right_back_I);
    }
  else
    {
      controlling=0;
      ro->motor_L = LEFT_Y;
      ro->motor_R = RIGHT_Y;
    }
      
  
#else

    /* Drive control.  It's open loop, meaning we just take the value from the joysticks
     *  without computation.  (The human closes the loop) */
    ro->motor_L = LEFT_Y;
    ro->motor_R = RIGHT_Y;

    /* Sometimes you want to pretend the robot is facing the opposite direction while
     *  driving.  We make that happen.  */

#endif
    if(REVERSE_INPUT)
      {
	tmp = ro->motor_L;
	ro->motor_L = REVERSE_PWM(ro->motor_R);
	ro->motor_R = REVERSE_PWM(tmp);
      }
    if(GEAR_SHIFT)
      {
	if(!already_shifted)
	  {
	    if(SHIFT > 0)
	      SHIFT = 0;
	    else
	      SHIFT = 255;
	    already_shifted=1;
	  }
      }
    else
      already_shifted=0;
    /*if(!ISNEAR(p3_y,127))
      {
	if(POS1_SW)
	  rt->arm_pos=POS1;
	if(POS2_SW)
	  rt->arm_pos=POS2;
	if(POS3_SW)
	  rt->arm_pos=POS3;
	if(POS4_SW)
	  rt->arm_pos=POS4;
	Set_Arm_Pos(rs,rt,ro);
      }
    else
      ro->arm = p3_y;
    */
    ro->arm = p3_y;
      
}

/*******************************************************************************
 * FUNCTION NAME: control
 * PURPOSE:       For autonomous control of the robot.  Decides on output values to make
 *                       the robot's current state go to the target state.
 * CALLED FROM:   Our_Autonomous_Code
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void control(robo_state * rs, robo_target * rt, robo_out * ro)
{
  static struct pidstate right,left;
  static int init=0;
  int tmp;
  if(init==0)
    {
      init++;
      init_controller(&left,P_GAIN_L_N,P_GAIN_L_D,I_GAIN_L_N,I_GAIN_L_D,D_GAIN_L_N,D_GAIN_L_D,127,-127);
      init_controller(&right,P_GAIN_R_N,P_GAIN_R_D,I_GAIN_R_N,I_GAIN_R_D,D_GAIN_R_N,D_GAIN_R_D,127,-127);
    }
  // right and left drive wheels
  /* We store the last outputs for PID controlled motors, so we know what change 
   *  to make.  We then replace the old last value with the one we're about to send out,
   *  and set the value to send out to its new value. */
  //  ro->last_out_R = pd_vel_control(RIGHT_SIDE,ro->last_out_R,rt->vel_R, rs->vel_R);
  //ro->last_out_L = pd_vel_control(LEFT_SIDE ,ro->last_out_L,rt->vel_L, rs->vel_L);

  ro->last_out_L = controller(&left,ro->last_out_L,rt->vel_L);
  ro->last_out_R = controller(&right,ro->last_out_R,rt->vel_R);
  //printf("control output L=%d R=%d\n", (int)ro->last_out_L, (int)ro->last_out_R) ;
  
  if(ro->last_out_R >= 254)
    ro->last_out_R = 254;
  else if (ro->last_out_R <= 127)
    ro->last_out_R = 127;

  if(ro->last_out_L >= 254)
    ro->last_out_L = 254;
  else if (ro->last_out_L <= 127)
    ro->last_out_L = 127;

  if(rt->vel_R <= 0)		// do not let output go negative
    ro->last_out_R = 127;

  if(rt->vel_L <= 0)		// do not let output go negative
    ro->last_out_L = 127;

   // set output
  ro->motor_R = (unsigned char)ro->last_out_R;
  ro->motor_L = (unsigned char)ro->last_out_L;
 
  if(rt->reverse_drive)
    {
      ro->motor_L = 255-ro->motor_L;
      ro->motor_R = 255-ro->motor_R;
    }

  Set_Arm_Pos(rs,rt,ro);
  //printf("control output motor L=%d R=%d\n", (int)ro->motor_L, (int)ro->motor_R) ;
}

void guidance(robo_state *rs, robo_target *rt)
{
  static int laststep=-1;
  static int step=0;
  static int traj=T_NULL;
  bot_params bp;

  traj = 0; //T_TURNSHOOT;
  if(step!=laststep)
    {
      switch(step)
	{
	  /* Use for sample. 
case 0:
	  traj=T_STRAIGHT;
	  bp.straight.velocity = 72; // 6ft/sec
	  bp.straight.heading = 0; 
	  bp.straight.time = 2500;
	  break;
	case 1:
	  traj=T_SPIRAL;
	  bp.intval.val = REVERSE;
	  break;
	case 2:
	  traj=T_ACQUISITOR;
	  bp.intval.val = REVERSE;
	  break;
	default:
	  rt->reverse_drive=0;
	  traj=T_NULL;
	  bp.time.val=-1;
	  break;
	  */
	}
      traj_array[traj].start(rs, &bp);
      laststep=step;
    }
  if(traj_array[traj].run(rs, rt) == TRAJ_DONE)
    {
      traj_array[traj].stop();
      step++;
    }
}

void output(robo_out * ro)
{
#ifdef TWO_MOTOR
  LEFT_DRIVE  = ro->motor_L;
  RIGHT_DRIVE = 255-ro->motor_R; // flip right motor
#else
  LEFT1_DRIVE = LEFT2_DRIVE = ro->motor_L;
  RIGHT1_DRIVE = RIGHT2_DRIVE = 255-ro->motor_R;
#endif
  ARM_MOTOR = ro->arm;
}

void update_state( robo_state * rs, robo_target *rt)
{
  rs->rpm_1k_R = get_rpm_1k(RIGHT_SIDE);
  rs->rpm_1k_L = get_rpm_1k(LEFT_SIDE);	

  rs->vel_R = (float)rs->rpm_1k_R / 1000.0 * WHEEL_CIRC_R / 60.0;
  rs->vel_L = (float)rs->rpm_1k_L / 1000.0 * WHEEL_CIRC_L / 60.0;
	
  if(rt->reverse_drive)
    {
      float tmp = rs->vel_L;
      rs->vel_L = rs->vel_R;
      rs->vel_R = tmp;
    }

  rs->pos_R += rs->vel_R * DELTA_T;
  rs->pos_L += rs->vel_L * DELTA_T;

  rs->cam_tracking = (T_Packet_Data.my != 0);
  rs->cam_pan_angle = PAN_ANGLE;
	
  rs->heading = Get_Gyro_Angle();

  rs->x_pos = 0;
  rs->y_pos = 0;

#ifdef USING_CURRENT_SENSORS
  rs->left_front_I = get_current(LF_CURRENT);
  rs->left_back_I = get_current(LB_CURRENT);
  rs->right_front_I = get_current(RF_CURRENT);
  rs->left_back_I = get_current(RB_CURRENT);
#endif

  rs->arm_pos = ARM_POT;
}

void robo_reset( robo_state * rs, robo_target * rt, robo_out *ro)
{
  rs->rpm_1k_R = 0;
  rs->rpm_1k_L = 0;
  
  rs->vel_R = 0;
  rs->vel_L = 0;
  
  rs->pos_R =0;
  rs->pos_L = 0;

  rs->arm_pos = 0;
  
  Reset_Gyro_Angle();
  rs->heading = 0;
  
  rs->x_pos = 0;
  rs->y_pos = 0;

  rt->heading = 0;
  
  rt->reverse_drive = OFF;

  rt->vel_R = 0;
  rt->vel_L = 0;

  rt->arm_pos = 0;
  
  rt->rpm_1k_L = 0;
  rt->rpm_1k_R = 0;
  
  ro->motor_L = 127;
  ro->motor_R = 127;
  
  ro->last_out_L = 127;
  ro->last_out_R = 127;

  ro->arm = 127;
}

void LED_Reset()
{
  Pwm1_red = Pwm2_red = Relay1_red = Relay2_red = Pwm1_green = Pwm2_green = Relay1_green = Relay2_green = 0;
}


int abs(int n)
{
  return (n>0 ? n : -n);
}

void turn_to_camera(robo_out *ro)
{
  int error;
  int turnspeed;
  static struct pidstate s;
  static int init = 0;
  if(init==0)
    {
      init=1;
      init_controller(&s,1,3,1,135,0,1,127,-127);
    }
  
  if(T_Packet_Data.my != 0)
    {
      error = PAN_SERVO - PAN_CENTER_PWM_DEFAULT;
      turnspeed=controller(&s,error,0);
      ro->motor_L = 127 - turnspeed;
      ro->motor_R = 127 + turnspeed;
    }
  else
    {
      init=0;
      ro->motor_L = 127;
      ro->motor_R = 127;
    }
}

/*******************************************************************************
 * FUNCTION NAME: Set_Arm_Pos
 * PURPOSE:       Calculates a speed to set the arm motor to using a PID controller.
                  The speed is stored in the robo_out struct ro.  
 * CALLED FROM:   control
 * ARGUMENTS:     a robo_state* with the current state of the robot, a robo_target*
                  with the desired arm position, and a robo_out* to store the motor
                  speed in
 * RETURNS:       void
 *******************************************************************************/
void Set_Arm_Pos(robo_state *rs, robo_target *rt, robo_out *ro)
{
  static struct pidstate ps;
  init_controller_divisor(&ps,ARM_P_GAIN,ARM_I_GAIN,0,127,-127);

  ro->arm = controller(&ps,rs->arm_pos,rt->arm_pos);
}
  
  
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
