/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "adc.h"
#include "gyro.h"
#include "ir.h"
#include "common_def.h"
#include "arm.h"

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/

unsigned char Old_Port_B = 0xFF;
extern char ir;
extern long gyro_angle;

/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow(void)     
{   
  unsigned char Port_B;
  unsigned char Port_B_Delta;       
  unsigned char int_byte;
	
  if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) // timer 0 interrupt?
    {
      INTCONbits.TMR0IF = 0; // clear the timer 0 interrupt flag [89]
      Timer_0_Int_Handler(); // call the timer 0 interrupt handler (in interrupts.c)
    }
  else if (PIR1bits.TMR1IF && PIE1bits.TMR1IE) // timer 1 interrupt?
    {
      PIR1bits.TMR1IF = 0; // clear the timer 1 interrupt flag [92]
      Timer_1_Int_Handler(); // call the timer 1 interrupt handler (in interrupts.c)
    }  
  else if (PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
    {
      PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
      Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in interrupts.c)
    }  
  else if (PIR2bits.TMR3IF && PIE2bits.TMR3IE) // timer 3 interrupt?
    {
      PIR2bits.TMR3IF = 0; // clear the timer 3 interrupt flag [93]
      Timer_3_Int_Handler(); // call the timer 3 interrupt handler (in interrupts.c)
    }  
  else if (PIR3bits.TMR4IF && PIE3bits.TMR4IE) // timer 4 interrupt?
    {
      PIR3bits.TMR4IF = 0; // clear the timer 4 interrupt flag [94]
      Timer_4_Int_Handler(); // call the timer 4 interrupt handler (in interrupts.c)
    }  
  else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // external interrupt 1?
    { 
      INTCON3bits.INT2IF = 0; // clear the interrupt flag [91]
      Int_1_Handler(); // call the interrupt 1 handler (in interrupts.c)
    }
  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // external interrupt 2?
    {
      INTCON3bits.INT3IF = 0; // clear the interrupt flag [91]
      Int_2_Handler(); // call the interrupt 2 handler (in interrupts.c)
    }
  else if (INTCONbits.RBIF && INTCONbits.RBIE) // external interrupts 3 through 6?
    {
      Port_B = PORTB; // remove the "mismatch condition" by reading port b            
      INTCONbits.RBIF = 0; // clear the interrupt flag [89]
      Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
      Old_Port_B = Port_B; // save a copy of port b for next time around
	 
      if(Port_B_Delta & 0x10) // did external interrupt 3 change state?
	{
	  Int_3_Handler(Port_B & 0x10 ? 1 : 0); // call the interrupt 3 handler (in interrupts.c)
	}
      if(Port_B_Delta & 0x20) // did external interrupt 4 change state?
	{
	  Int_4_Handler(Port_B & 0x20 ? 1 : 0); // call the interrupt 4 handler (in interrupts.c)
	}
      if(Port_B_Delta & 0x40) // did external interrupt 5 change state?
	{
	  Int_5_Handler(Port_B & 0x40 ? 1 : 0); // call the interrupt 5 handler (in interrupts.c)
	}
      if(Port_B_Delta & 0x80) // did external interrupt 6 change state?
	{
	  Int_6_Handler(Port_B & 0x80 ? 1 : 0); // call the interrupt 6 handler (in interrupts.c)
	}
    }	
  else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
    {
      PIR1bits.ADIF = 0; // clear the ADC interrupt flag
      ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
    }       
  else if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
    {
#ifdef ENABLE_SERIAL_PORT_ONE_RX
      Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
#endif
    }                              
  else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
    {
#ifdef ENABLE_SERIAL_PORT_TWO_RX
      Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
#endif
    } 
  else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
    {
#ifdef ENABLE_SERIAL_PORT_ONE_TX
      Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
#endif
    }                              
  else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
    {
#ifdef ENABLE_SERIAL_PORT_TWO_TX
      Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
#endif
    }


  return;


	if(PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if(PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if(PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if(PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}                     
	else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}       
	else if(INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
	{
		INTCON3bits.INT2IF = 0;
	}
	else if(INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
	{
		INTCON3bits.INT3IF = 0;
	}
	else if(INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
	{
		int_byte = PORTB;		/* You must read or write to PORTB		*/
		INTCONbits.RBIF = 0;	/* and clear the interrupt flag			*/
	}							/* to clear the interrupt condition.	*/
}

/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void){
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  static long t = 0;

  pwm01 = pwm02 = pwm03 = pwm04 = pwm11 = pwm12 = 127;

  relay4_fwd=relay4_rev=relay5_fwd=relay5_rev=relay6_fwd=relay6_rev=relay7_fwd=relay7_rev=0;

  //these shouldn't be necessary, but I had some problems
  init_ir();
  init_arm();
  init_gyro_pids();
  //printf("User_Autonomous_Code\n");

  /*
  Reset_Gyro_Angle();
  if(CORNER_SWITCH) //starting in corner position
    gyro_angle += 785;
  */


  while (autonomous_mode){   /* DO NOT CHANGE! */
    Process_Data_From_Local_IO();
    if (statusflag.NEW_SPI_DATA){      /* 26.2ms loop area */
      Getdata(&rxdata); /* DO NOT DELETE, or you will be stuck here forever! */

      t++;
      ir = check_ir();

      if(t > 190 && AUTON_ON){
	if(CORNER_SW_UP)
	  Gyro_Drive();
	else if(CORNER_SW_DOWN || CORNER_SW_MID)
	  IR_Drive();
      }
      else
	L_DRIVE1 = L_DRIVE2 = R_DRIVE1 = R_DRIVE2 = ARM = STEER = 127;

      Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
      Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void){
  /* Add code here that you want to be executed every program loop. */
  // new ADC data available?
  if(Get_ADC_Result_Count()){
    Process_Gyro_Data();
    Reset_ADC_Result_Count();
  }	
}
