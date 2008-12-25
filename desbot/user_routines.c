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
#include "serial_ports.h"
#include "adc.h"
#include "gyro.h"
#include "controller.h"
#include "interrupts.h"
#include "velocity.h"
#include "common_def.h"
#include "ir.h"
#include "arm.h"
//#include "laws.h"
#include "gyrodrive.h"

extern long gyro_angle;
extern int left_ticks, right_ticks;
extern unsigned char aBreakerWasTripped;
extern struct pidstate armpid;

extern char ir;

extern rx_data_record rxdata;
long time_left;

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
/*
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}
*/

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
/*
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value){
  if (switch_state == CLOSED){ 
    if(*input_value < 127)
      *input_value = 127;
  }
}
*/

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
unsigned char Limit_Mix (int intermediate_value){
  static int limited_value;
  
  if (intermediate_value < 2000)
    limited_value = 2000;
  else if (intermediate_value > 2254)
    limited_value = 2254;
  else
    limited_value = intermediate_value;
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
void User_Initialization(void){
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

  digital_io_01 = digital_io_02 = INPUT; //wheel counters
  digital_io_03 = INPUT; //pressure regulator
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT; //IR
  digital_io_11 = digital_io_12 = digital_io_13 = INPUT; //corner switches
  digital_io_17 = digital_io_18 = OUTPUT; //prog pins

  L_DRIVE1 = L_DRIVE2 = R_DRIVE1 = R_DRIVE2 = ARM = STEER = 127;
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
  init_gyro_pids();
  init_ir();
  init_arm();

  Init_Serial_Port_One();
  //Init_Serial_Port_Two();
  Initialize_Interrupts();

  stdout_serial_port = SERIAL_PORT_ONE;

  Initialize_Gyro();
  Initialize_ADC();

  record_reset();
 
  Putdata(&txdata);             /* DO NOT CHANGE! */
  //printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */
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
  static unsigned int i = 0;
  static unsigned int j = 0;
  int temp_gyro_rate;
  long temp_gyro_angle;
  int temp_gyro_bias;
  static char last_was_display = 0;
  //static char progout = 1;
  
  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

  ir = check_ir();

  i++;
  j++; // this will rollover every ~1000 seconds

  /*
  if(j == 10){
    printf("\rCalculating Gyro Bias...");
  }
  */
  if(j == 60){
    // start a gyro bias calculation
    Start_Gyro_Bias_Calc();
  }

  //turn on lights in sequence; looks cool and is informative
  //if(j >= 60)
  //Pwm1_green = 1;
  if(j >= 80)
    Pwm1_red = 1;
  //if(j >= 100)
  //Pwm2_green = 1;
  if(j >= 120)
    Pwm2_red = 1;
  //if(j >= 140)
  //Relay1_green = 1;
  if(j >= 160)
    Relay1_red = 1;
  //if(j >= 180)
  //Relay2_green = 1;
  if(j >= 200)
    Relay2_red = 1;

  if(j == 200){
    // terminate the gyro bias calculation
    Stop_Gyro_Bias_Calc();
    // reset the gyro heading angle
    Reset_Gyro_Angle();
    //printf("Done\r");
  }

  if(i >= 30 && j >= 200){
    temp_gyro_bias = Get_Gyro_Bias();
    temp_gyro_rate = Get_Gyro_Rate();
    temp_gyro_angle = Get_Gyro_Angle();
    
    printf(" Gyro Bias=%d\r\n", temp_gyro_bias);
    printf(" Gyro Rate=%d\r\n", temp_gyro_rate);
    printf("Gyro Angle=%d\r\n\r\n", (int)temp_gyro_angle);
		
    //printf("p1_y: %d, p1_x: %d\r\n",(int)p1_y,(int)p1_x);
    //printf("p2_y: %d, p2_x: %d\r\n",(int)p2_y,(int)p2_x);
    //printf("p3_y: %d, p3_x: %d\r\n",(int)p3_y,(int)p3_x);
    //printf("p4_y: %d, p4_x: %d\r\n",(int)p4_y,(int)p4_x);

    i = 0;
  }
  printf("left %d right %d\n", left_ticks, right_ticks);

  //if(j >= 200)
  //IR_Drive();

  EEPROM_Write_Handler();
  EEPROM_Write_Handler();


  if(j >= 200)
    Default_Routine();

  if(user_display_mode && !last_was_display) //just went into user mode? reset timer
    time_left = 1200000;
  last_was_display = user_display_mode;

  /*
  //prog if remote down pressed
  progout &= (ir != 2);
  
  rc_dig_out17 = 0;
  rc_dig_out18 = progout;
  */

  Putdata(&txdata);
}

void do_turn(int speed, int amt){
#ifdef USE_ACK //defined (or not) in user_routines.h
  //right, left
  //467, 659
  //-89, 103
#define MAX 95 // 103
#define MIN 90 // 89 //absolute value of min - center

  char c;
  unsigned int pot = Get_Analog_Value(rc_ana_in03);

  //NO DIFFERENTIAL!
  R_DRIVE1 = R_DRIVE2 = (unsigned char)(127 - speed);
  L_DRIVE1 = L_DRIVE2 = (unsigned char)(127 + speed);

  //adjust for asymmetry in readings
  if(amt > 0)
    amt = amt * MAX / 127;
  else
    amt = amt * MIN / 127;

  c = (char)controller(&steerpid, pot, amt + STEER_CENTER);

  //steering
  STEER = (unsigned char)(127 - c);
#else
  L_DRIVE1 = L_DRIVE2 = Limit_Mix(2000 + 127 + speed - amt);
  R_DRIVE1 = R_DRIVE2 = Limit_Mix(2000 + 127 - speed - amt);
#endif
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void){
  int steergoto = (int)X_DRIVE_AXIS - 127;
  int speed = (int)Y_DRIVE_AXIS - 127;

  static int t = 0;

  if(speed > -10 && speed < 10) //decrease sensitivity of turning at low speed
    steergoto = steergoto * 4 / 5;

  //handle tilt/elbow position
  do_arm();
  //steering

  if(p1_sw_trig && p2_sw_trig){ //both triggers pressed? tank drive
    L_DRIVE1 = L_DRIVE2 = p2_y;
    R_DRIVE1 = R_DRIVE2 = 255-p1_y;
  }
  else
    do_turn(speed, steergoto);

  //pump
  PUMP_F = !P_SWITCH;
  PUMP_R = 0;

  GRAB_F = GRAB_BUTTON;

  do_puncher();

  printf("digin11 %d digin12 %d digin13 %d\n", rc_dig_in11, rc_dig_in12, rc_dig_in13);
  printf("armpot %d\n", Get_ADC_Result(ARM_POT));
  //printf("regulator: %d pump %d\n", P_SWITCH, PUMP_F);
  //printf("ir %d\n", ir);

  User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */

#define DEAD 4
  if(L_DRIVE1 >= 127 + DEAD)
    L_DRIVE1 = L_DRIVE2 -= DEAD;
  else if(L_DRIVE2 <= 127 - DEAD)
    L_DRIVE1 = L_DRIVE2 += DEAD;
  else
     L_DRIVE1 = L_DRIVE2 = 127;

  if(R_DRIVE1 >= 127 + DEAD)
    R_DRIVE1 = R_DRIVE2 -= DEAD;
  else if(R_DRIVE2 <= 127 - DEAD)
    R_DRIVE1 = R_DRIVE2 += DEAD;
  else
    R_DRIVE1 = R_DRIVE2 = 127;

  //follow_three_laws();

  /*
  //whee
  t++;
  Pwm1_green =   t & 1;
  Pwm1_red =     (t & 2) >> 1;
  Pwm2_green =   (t & 4) >> 2;
  Pwm2_red =     (t & 8) >> 3;
  Relay1_green = (t & 16) >> 4;
  Relay1_red =   (t & 32) >> 5;
  Relay2_green = (t & 64) >> 6;
  Relay2_red =   (t & 128) >> 7;
  */

  /*
  t++;
  Pwm1_green = Pwm2_red = Relay1_green = Relay2_red = (t & 16) >> 4;
  Pwm1_red = Pwm2_green = Relay1_red = Relay2_green = !((t & 16) >> 4);
  */

  /*
  time_left -= 262;

  if(time_left <= 70000)
    Pwm1_green = 1;
  if(time_left <= 60000)
    Pwm2_green = 1;
  if(time_left <= 50000)
    Relay1_green = 1;
  if(time_left <= 40000)
    Relay2_green = 1;
  if(time_left <= 30000)
    Switch1_LED = 1;
  if(time_left <= 20000)
    Switch2_LED = 1;
  if(time_left <= 10000)
    Switch3_LED = 1;
  */

  if(p4_sw_trig)
    record();
  if(p3_sw_trig)
    replay();
  if(p2_sw_trig)
    record_reset();

} /* END Default_Routine(); */
//jay:  6463399798
//meed: 9178807974

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
