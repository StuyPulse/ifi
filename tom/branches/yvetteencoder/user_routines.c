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
#include "trajectory.h"

#include "pwm.h"
/* HEADERS by us */
#include "velocity.h"
#include "controller.h"
#include "common_def.h"

#include "arm_control.h"

/* These are variables that control our PID controllers */
extern volatile unsigned int left_clock;
extern volatile unsigned int right_clock;
extern volatile int left_vel;
extern volatile int right_vel;
extern volatile int left_ticks;
extern volatile int right_ticks;
extern volatile unsigned int left_index;
extern volatile unsigned int right_index;

extern volatile trajectory traj_array[];

/* Structures for the robot guidance and controller systems */
robo_state r_state;
robo_target r_target;
robo_out r_out;

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
  //Initialize_Gyro();

  Initialize_Interrupts();
  //Initialize_Timer_0();     DO NOT USE TIMER0 - IT PROVIDES SOME BENEFIT THAT WE SHOULD AVOID LOSING.
  Initialize_Timer_1();
  //  Initialize_Timer_2(); //Using ADC timer2 instead.
  //  Initialize_Timer_3(); Do not use this, Kevin's PWM code does
  Initialize_Timer_4();   

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

  // This calibrates the gyro as soon as the robot turns on.
  // It takes about 11 seconds, during which the robot is disabled.
  static unsigned int j=300;
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
	Our_User_Code();
	//printf("pwm01: %d\t pwm02: %d\t pwm03: %d\t pwm04: %d\r\n",
	//pwm01,pwm02,pwm03,pwm04);

      //Tracking_Info_Terminal();
	
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
  static unsigned long time=0;
  //Camera_Handler();
  //Servo_Track();
  update_state(&r_state,&r_target);
  //static char stage=0;

  /* We use our guidance system to figure out what physical state we want the bot */
  guidance(&r_state, &r_target);

  /* Then, the controller figures out what needs to put in our outputs to meet out 
   *  target state */
  r_target.arm_pos = -1;
  control(&r_state, &r_target, &r_out);
      
  /* And finally sets the outputs to the values we decide they should be at. */
  output(&r_out);
  time++;
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
  
  Servo_Track();  

  update_state(&r_state,&r_target);
 
  /* Standard drive runs whatever code we want during user mode.
   *  Because parts of it may be closed-loop, it is in the same framework
   *  we use for autonomous. */
  Standard_Drive(&r_state, &r_target, &r_out);

  /*  if(p1_sw_trig)
      Our_Autonomous_Code();
  */
  /* Finally, we send the data out to the motors and other outputs. */
  output(&r_out);  

  if(user_display_mode)
    {
     
      User_Mode_byte = backup_voltage*10;
    }
 
  Pwm1_red = rc_dig_in01;
  Pwm2_red = rc_dig_in02;
  
  //printf("PosL: %d, PosR: %d\r\n",(int)r_state.pos_L,(int)r_state.pos_R);
  //printf("VelL: %d, VelL: %d\r\n",(int)r_state.vel_L,(int)r_state.vel_R);


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
  static char expcontrol = 1;

  /* Drive control.  It's open loop, meaning we just take the value
   *  from the joysticks without computation.  (The human closes the
   *  loop) 
   */

  if(p3_sw_aux2)
    {
      expcontrol=0;
      Relay1_green = 1;
      Relay1_red = 0;
    }
  else if((ISNEAR(p3_aux,143)))
    {
      expcontrol=1;
      Relay1_red =1;
      Relay1_green = 0;
    }
  ro->motor_L = LEFT_Y;//joymap(LEFT_Y,expcontrol);
  ro->motor_R = RIGHT_Y;//joymap(RIGHT_Y,expcontrol);

  // Figure out what gear we want to be in from the user, then shift
  Gear_Input(rs,rt);
  Shift_Gear(rs,rt,ro);

  if(REVERSE_SW)
    rt->reverse_drive=1;
  else if(FORWARD_SW)
    rt->reverse_drive=0;
  if(rt->reverse_drive)
    {
      tmp = ro->motor_L;
      ro->motor_L = REVERSE_PWM(ro->motor_R);
      ro->motor_R = REVERSE_PWM(tmp);
    }


  //Jordan's Ramp Servo code
  Deploy_Ramps(ro,LEFT_RAMP_DEPLOY,RIGHT_RAMP_DEPLOY);
  User_Arm_Control(rs,rt,ro);

  if(LANCE_RAISE)
    ro->lance_motor = 127+LANCESPEED;
  else if(LANCE_LOWER)
    ro->lance_motor = 127-LANCESPEED;
  else
    ro->lance_motor = 127;



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
  ro->last_out_R = pd_vel_control(RIGHT_SIDE,ro->last_out_R,rt->vel_R, rs->vel_R);
  ro->last_out_L = pd_vel_control(LEFT_SIDE ,ro->last_out_L,rt->vel_L, rs->vel_L);

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
  if(rt->arm_pos >= 0)
    Set_Arm_Pos(rs,rt,ro);
  else
    ro->arm = 127;
  Shift_Gear(rs,rt,ro);
  
}

void guidance(robo_state *rs, robo_target *rt)
{
  static int laststep=-1;
  static int step=0;
  static int traj=0;
  bot_params bp;

  traj = T_STRAIGHT_NOGYRO;
  if(step!=laststep)
    {
      switch(step)
	{

	case 0:
	  traj=T_STRAIGHT_NOGYRO;
	  bp.straight.distance=0;
	  bp.straight.time=20000;
	  bp.straight.heading=0;
	  bp.straight.velocity=30;
	  bp.straight.gear=1; 
	  
	  break;
	default:
	  return TRAJ_DONE;
	  break;
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

  
  //Ramp stuff
  L_RAMP_SERVO = ro->ramp_servo_l;
  R_RAMP_SERVO = ro->ramp_servo_r;

  SHIFT=ro->shift_servo;
  LANCE_MOTOR = ro->lance_motor;
}

void update_state( robo_state * rs, robo_target *rt)
{
  static unsigned int old_r_ticks=0;
  static unsigned int old_l_ticks=0;


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
  rs->pos_R += (float) (r_out.motor_R > 127 ? 1 : -1) * (right_ticks - old_r_ticks) * WHEEL_CIRC_R / NUM_DRIVE_SPOKE;
  rs->pos_L += (float) (r_out.motor_L > 127 ? 1 : -1) * (left_ticks - old_l_ticks) * WHEEL_CIRC_L / NUM_DRIVE_SPOKE;
  old_r_ticks = right_ticks;
  old_l_ticks = left_ticks;


  //rs->cam_tracking = (T_Packet_Data.my != 0);
	
  rs->heading = Get_Gyro_Angle();

  rs->cam_tilt_angle = TILT_ANGLE_RADS;
  rs->cam_pan_angle = PAN_ANGLE_RADS;

  rs->cam_pan_servo = PAN_SERVO;
  rs->cam_tilt_servo = TILT_SERVO;

  rs->x_pos = 0;
  rs->y_pos = 0;

#ifdef USING_CURRENT_SENSORS
  rs->left_front_I = get_current(LF_CURRENT);
  rs->left_back_I = get_current(LB_CURRENT);
  rs->right_front_I = get_current(RF_CURRENT);
  rs->left_back_I = get_current(RB_CURRENT);
#endif

  //rs->arm_pos = Get_Analog_Value(ARM_POT);
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

  rs->right_ramp = UNDEPLOYED;
  rs->left_ramp = UNDEPLOYED;
 
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

  rt->right_ramp = UNDEPLOYED;
  rt->left_ramp = UNDEPLOYED;

  ro->ramp_motor_l = 127;
  ro->ramp_motor_r = 127;
  ro->ramp_servo_l = RAMP_SERVO_UNDEPLOYED;
  ro->ramp_servo_r = RAMP_SERVO_UNDEPLOYED;
  
  ro->motor_L = 127;
  ro->motor_R = 127;
  
  ro->last_out_L = 127;
  ro->last_out_R = 127;

  ro->arm = 127;

  ro->shift_servo = 0;
}

void LED_Reset()
{
  Pwm1_red = Pwm2_red = Relay1_red = Relay2_red = Pwm1_green = Pwm2_green = Relay1_green = Relay2_green = 0;
}


int abs(int n)
{
  return (n>0 ? n : -n);
}

int turn_to_camera(robo_out *ro)
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
      return error;
    }
  else
    {
      init=0;
      ro->motor_L = 127;
      ro->motor_R = 127;
    }
}


void Shift_Gear(robo_state *rs, robo_target *rt, robo_out *ro)
{
  switch(rt->gear)
    {
    case LOW:
      ro->shift_servo = LOW_SERVO_VALUE;
      rs->gear = LOW;
      break;
    case MIDDLE:
      ro->shift_servo = MIDDLE_SERVO_VALUE;
      rs->gear = MIDDLE;
      break;
    case HIGH:
      ro->shift_servo = HIGH_SERVO_VALUE;
      rs->gear = HIGH;
      break;
    }

}

void Gear_Input(robo_state *rs, robo_target *rt)
{
  if(HIGH_SW)
    rt->gear = HIGH;
  else if(LOW_SW)
    {
      rt->gear = LOW;
      rs->gear = LOW;
    }
    
  if(rs->gear == MIDDLE || rs->gear == HIGH)
    {
      if(HIGH_GEAR_TOGGLE)
	{
	  rt->gear = HIGH;
	}
      else
	{
	  rt->gear = MIDDLE;
	}
    }

}

void Deploy_Ramps(robo_out *ro,int left, int right)
{
  if(left)
    ro->ramp_servo_l = RAMP_SERVO_DEPLOYED;
  else
    ro->ramp_servo_l = RAMP_SERVO_UNDEPLOYED;
  
  if(right)
    ro->ramp_servo_r = RAMP_SERVO_DEPLOYED;
  else
    ro->ramp_servo_r = RAMP_SERVO_UNDEPLOYED;
}
  
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
