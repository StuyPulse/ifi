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
 *******************************************************************************/

#include <stdio.h>
#include <math.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "adc.h"
#include "camera.h"
#include "serial_ports.h"
#include "tracking.h"
#include "gyro.h"
#include "interrupts.h"
#include "velocity.h"
#include "controller.h"
#include "common_def.h"
#include "trajectory.h"

extern volatile trajectory traj_array[];
extern volatile unsigned int left_clock;
extern volatile unsigned int right_clock;
extern volatile unsigned int fly_clock;
extern volatile int left_vel;
extern volatile int right_vel;
extern volatile int fly_count;
extern volatile unsigned int left_index;
extern volatile unsigned int right_index;

int auton_pot;

robo_state r_state;
robo_target r_target;
robo_out r_out;

#define ISNEAR(in,x) (in >= (x-5) && in <= (x+5) )

#define OUR_DEBUG
#define USE_OI_POTS
//#define USE_JOYSTICK_DEADZONE


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
   unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
   unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
   int           angle_deviation = 142; (can vary from -32,768 to 32,767)
   unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/


/*******************************************************************************
 * FUNCTION NAME: Limit_Switch_Max
 * PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
 *                limit switch is on.
 * CALLED FROM:   this file
 * ARGUMENTS:     
 *     Argument       Type             IO   Description
 *     --------       -------------    --   -----------
 *     switch_state   unsigned char    I    limit switch state
 *     *input_value   pointer           O   points to PWM byte value to be limited
 * RETURNS:       void
 *******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
    { 
      if(*input_value > 127)
	*input_value = 127;
    }
}


/*******************************************************************************
 * FUNCTION NAME: Limit_Switch_Min
 * PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
 *                limit switch is on.
 * CALLED FROM:   this file
 * ARGUMENTS:     
 *     Argument       Type             IO   Description
 *     --------       -------------    --   -----------
 *     switch_state   unsigned char    I    limit switch state
 *     *input_value   pointer           O   points to PWM byte value to be limited
 * RETURNS:       void
 *******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
    { 
      if(*input_value < 127)
	*input_value = 127;
    }
}


/*******************************************************************************
 * FUNCTION NAME: Limit_Mix
 * PURPOSE:       Limits the mixed value for one joystick drive.
 * CALLED FROM:   Default_Routine, this file
 * ARGUMENTS:     
 *     Argument             Type    IO   Description
 *     --------             ----    --   -----------
 *     intermediate_value    int    I    
 * RETURNS:       unsigned char
 *******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
    {
      limited_value = 2000;
    }
  else if (intermediate_value > 2254)
    {
      limited_value = 2254;
    }
  else
    {
      limited_value = intermediate_value;
    }
  return (unsigned char) (limited_value - 2000);
}


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


  Init_Serial_Port_One();
  Init_Serial_Port_Two();
  // Initialize interrupts and timers
  Initialize_ADC();
  Initialize_Gyro();

  Initialize_Interrupts();
  //Initialize_Timer_0();     DO NOT USE TIMER0 - IT PROVIDES SOME BENEFIT THAT WE SHOULD AVOID LOSING.
  Initialize_Timer_1();
  //  Initialize_Timer_2(); //Using ADC timer2 instead.
  Initialize_Timer_3();	
  Initialize_Timer_4();   

  PAN_SERVO = PAN_CENTER_PWM_DEFAULT;
  TILT_SERVO = TILT_MIN_PWM_DEFAULT;

  robo_reset( &r_state, &r_target, &r_out);
  init_traj();

  Putdata(&txdata);             /* DO NOT CHANGE! */

  //  printf("%s\n", strptr);       /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
 * FUNCTION NAME: Process_Data_From_Master_uP
 * PURPOSE:       Executes every 26.2ms when it gets new data from the master 
 *                microprocessor.
 * CALLED FROM:   main.c
 * ARGUMENTS:     none
 * RETURNS:       void
 *******************************************************************************/
void Process_Data_From_Master_uP(void){
  /*	unsigned int i,min,max;
	min = -1;
	max = 0;
  */  

  //  static float pos_L = 0;
  //  static float pos_R = 0;
  //  float pos_error;
  //  long curr_des_L;
  //  long curr_des_R; 


/*
  int temp_gyro_rate;
  long temp_gyro_angle;
  int temp_gyro_bias;
*/

  static unsigned int i=0;
  static unsigned int j=0;
  i++;
  j++; // this will rollover every ~1000 seconds
  
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

  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
 

      //temp_gyro_bias = Get_Gyro_Bias();
      //temp_gyro_rate = Get_Gyro_Rate();
      //printf(" Gyro Bias=%d\r\n", temp_gyro_bias);
      //printf(" Gyro Rate=%d\r\n", temp_gyro_rate);
      
      //temp_gyro_angle = Get_Gyro_Angle();
      //printf("Gyro Angle=%d\r\n\r\n", (int)temp_gyro_angle);

  //Camera_Handler();
  //Servo_Track();
  //printf("Pan angle: %d MX: %d  MY: %d \r",(int)PAN_ANGLE,T_Packet_Data.my,T_Packet_Data.my);
  //vel_control_routine();
  //Default_Routine();
  //pwm04 = p3_y;   //Acquisitor
  //printf("right_in:%d, left_in:%d, right_clock:%d, left_clock:%d\r",(int)rc_dig_in01,(int)rc_dig_in02,right_clock,left_clock);

  //printf("%d %d\n",(int)left_clock,(int)right_clock);
  if ( j >= 300 )
    {
      if ( GORDON_OVERRIDE > 140 )
	{
	  //Our_Autonomous_Code() ;
	}
      else
	{
	  Our_User_Code();
	}
      
    }
  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

  Putdata(&txdata);             /* DO NOT CHANGE! */
}


/*******************************************************************************"****************
 ***********************************     OUR STUFF  *********************************************
 ***********************************************************************************************/

void Our_Autonomous_Code()
{
  static int i =0;
  Camera_Handler();
  Servo_Track();

  update_state(&r_state,&r_target);
  guidance(&r_state, &r_target);


  control(&r_state, &r_target, &r_out);
  output(&r_out);
}

void Our_User_Code()
{

  auton_pot = AUTONOMOUS_CHOOSER;
  
  Camera_Handler();
  Servo_Track();

  
  update_state(&r_state,&r_target);
  Standard_Drive(&r_state, &r_target, &r_out);
  output(&r_out);
  
}

void Standard_Drive(robo_state *rs, robo_target *rt, robo_out *ro)
{

  ro->motor_L = LEFT_Y;
  ro->motor_R = RIGHT_Y;

  
  if(REVERSE_INPUT)
    {
      tmp = ro->motor_L;
      ro->motor_L = REVERSE_PWM(ro->motor_R);
      ro->motor_R = REVERSE_PWM(tmp);
    }
  
  //aquisitor control

  LED_Control();
  
  //printf("spiral %d/%d flywheel %d/%d\r", SPIRAL_POT, SPIRAL, SHOOTER_POT, FLYWHEEL);
}

void control(robo_state * rs, robo_target * rt, robo_out * ro){

  //printf("control input L=%d,%d R=%d,%d\n",
  // (int)rt->vel_L, (int)rs->vel_L, (int)rt->vel_R, (int)rs->vel_R) ;

  // right and left drive wheels
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
  //printf("control output motor L=%d R=%d\n", (int)ro->motor_L, (int)ro->motor_R) ;


}

void guidance(robo_state *rs, robo_target *rt)
{
  static int laststep=-1;
  static int step=0;
  static int traj=T_NULL;
  bot_params bp;

  if(auton_pot<3)
    return;

  if(step!=laststep)
    {
      switch(step)
	{
	default:
	  rt->reverse_drive=0;
	  traj=T_NULL;
	  bp.time.val=-1;
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
  LEFT_DRIVE  = ro->motor_L;
  RIGHT_DRIVE = 255-ro->motor_R; // flip right motor
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
	
  //rs->heading += tan((rs->vel_R - rs->vel_L) / WHEEL_BASE) * DELTA_T;

  //  rs->looped=0;
  rs->heading = Get_Gyro_Angle();
  /*
    if(rs->heading < 0)
    {
    rs->looped = 1;
    rs->heading += 6283;
    }
    if(rs->heading >= 6283)
    {
    rs->looped = 1;
    rs->heading -= 6283;
    }
  */

  rs->x_pos = 0;
  rs->y_pos = 0;
}

void robo_reset( robo_state * rs, robo_target * rt, robo_out *ro)
{
  rs->rpm_1k_R = 0;
  rs->rpm_1k_L = 0;
  
  rs->vel_R = 0;
  rs->vel_L = 0;
  
  rs->pos_R =0;
  rs->pos_L = 0;
  
  Reset_Gyro_Angle();
  rs->heading = 0;
  
  rs->x_pos = 0;
  rs->y_pos = 0;

  rt->heading = 0;
  
  rt->reverse_drive = OFF;

  rt->vel_R = 0;
  rt->vel_L = 0;
  
  rt->rpm_1k_L = 0;
  rt->rpm_1k_R = 0;
  
  ro->motor_L = 127;
  ro->motor_R = 127;
  
  ro->last_out_L = 127;
  ro->last_out_R = 127;
}

void LED_Control()
{
  /* This is horrible spaghetti code; I'm really really sorry but it
     works and I'm too lazy to make it nicer */
  static int last_step = 0;
  static int delay=0;
  int tmp;
  int pan_error;
  //printf("Last: %d\n",last_step);

#define LED_ANGLE_STEP	( 5 )

  if(user_display_mode == 0)
    {
      if(T_Packet_Data.my == 0)
	{
	  if(delay>5)
	    {
	      if(last_step!=1)
		  {
		    last_step=1;
		    LED_Reset();
		    Pwm1_red=1;
		  }
	      delay = 0;
	      tmp = Relay2_red;
	      Relay2_red = Relay1_red;
	      Relay1_red = Pwm2_red;
	      Pwm2_red = Pwm1_red;
	      Pwm1_red = tmp;
	      last_step = 2;
	    }
	  else
	    delay++;
	}
      else 
	{
	  LED_Reset();
	  last_step=0;
	  if( PAN_ANGLE_DEGREES > 4*LED_ANGLE_STEP || PAN_ANGLE_DEGREES < -4*LED_ANGLE_STEP)
	    {
	      42;
	    }
	  else if((PAN_ANGLE_DEGREES > -1) && (PAN_ANGLE_DEGREES < 1))
	    {
	      Relay1_green=1;	      
	      Relay2_green=1;
	      Pwm1_green=1;
	      Pwm2_green=1;
	      Relay1_red=1;	      
	      Relay2_red=1;
	      Pwm1_red=1;
	      Pwm2_red=1;
	    }
	  else if(PAN_ANGLE_DEGREES > 0) 
	    {
	      if(PAN_ANGLE_DEGREES < LED_ANGLE_STEP )
		{
		  Pwm1_red=1;	      
		  Pwm2_red=1;
		  Relay1_red=1;
		}
	      else if(PAN_ANGLE_DEGREES < 2*LED_ANGLE_STEP)
		{
		  Pwm2_red=1;
		  Relay1_red=1;
		}
	      else if(PAN_ANGLE_DEGREES < 3*LED_ANGLE_STEP)
		{
		  Relay1_red=1;
		}
	      Relay2_red=1;
	    }

	  else if(PAN_ANGLE_DEGREES < 0) 
	    {
	      if(PAN_ANGLE_DEGREES > -LED_ANGLE_STEP )
		{
		  Pwm1_green=1;	      
		  Pwm2_green=1;
		  Relay1_green=1;
		}
	      else if(PAN_ANGLE_DEGREES > -2*LED_ANGLE_STEP)
		{
		  Pwm2_green=1;
		  Relay1_green=1;
		}
	      else if(PAN_ANGLE_DEGREES > -3*LED_ANGLE_STEP)
		{
		  Relay1_green=1;
		}
	      Relay2_green=1;
	    }
	}
    }
  else
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
}

void LED_Reset()
{
  Pwm1_red = Pwm2_red = Relay1_red = Relay2_red = Pwm1_green = Pwm2_green = Relay1_green = Relay2_green = 0;
}



int abs(int n)
{
  return (n * n) / n;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/