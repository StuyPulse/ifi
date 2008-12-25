#include "BuiltIns.h"
#include "controller.h"
#include "guidance.h"
#include "schemes.h"

#include <stdio.h>


void Initialize(void)
{
  printf("Initializing\n");
  //TwoWheelDrive(1,2);
}

void IO_Initialization(void)
{
  SetCompetitionMode(2);
}

void OperatorControl(void)
{
  struct robo_state r;
  unsigned int pot;
  struct pidstate s;
  unsigned int out;
  int error;
  TPacket *t;
  s.p_gain_num=2;
  s.p_gain_den=7;


  s.i_gain_num=2;
  s.i_gain_den=100;
  s.d_gain_num=0;
  s.d_gain_den=1;
  s.u_lim=127;
  s.l_lim=-127;
  s.error_sum=0;
  s.last_error=0;

  TwoWheelDrive(2,1);
  InitCamera(1);
  StartCamera();
  StartTimer(1);
  StartTimer(2);
  
  init_rs(&r,OI_SCHEME,RC_SCHEME,CONSTANTS);

  while(1)
    {
      t = CopyTrackingData();
      printf("timer 1: %d\ntimer 2:%d\n", GetTimer(1), GetTimer(2));
      Servo_Track(t);
      if(GetTimer(2)>26)
	{
	  printf("hello");
	  
	  PresetTimer(2,0);
	}
      if(GetTimer(1)>500)
	{
	  //t = CopyTrackingData();
	  print_tracking_data(t);
	  error = ((int) 127) - t->pan;
	  printf("pan: %d\n error: %d\n", t->pan, error);

	  if (t->confidence > 0){
	    Drive(0, error);
	    printf("error: %d\n", error);
	  }

	  PresetTimer(1,0);
	}
      //map_oi(&r);
    }
 
}

void Autonomous()
{}

void main(void)
{}
    
