#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "interrupts.h"
#include "velocity.h"
#include "common_def.h"
#include "controller.h"

int factor;

float pd_vel_control(int side, float last_out, float desired, float actual)
{
 
  //long error;
  float error ;
	 static float last_error_L = 0, last_error_R = 0, last_error_F = 0; 
     float out = last_out;
		
	  error = desired - actual; 
	  if(side == RIGHT_SIDE){
		  out = last_out + P_GAIN_R(error) ;//+ D_GAIN_R(last_error_R,error);
		  last_error_R = error;
	  }
	  else if(side == LEFT_SIDE){
		  out = last_out + P_GAIN_L(error) ;//+ D_GAIN_L(last_error_L,error);
		  last_error_L = error;
	  }
	  else if(side == FLY_SENSOR){
		  out = last_out + P_GAIN_F(error) ;//+ D_GAIN_F(last_error_F,error);
		  last_error_F = error;
	  }

/*	  if(out > 255)
		 out = 255;
	  if(out < 127)
		 out = 127;
*/

     // printf("get_velocity(): %ld.   get_rpm_1k(): %ld error: %ld P_GAIN(E): %d last_out: %d\r",get_velocity(1),get_rpm_1k(1)/1000,error,(int)P_GAIN(error),(int)last_out);
	  return out;
}

float ramp_feet(int inches, int ramptime)
{
  
}

float ramp_coefficient(int curr_time, int max_time, int ramp_time)
{
  if(curr_time < ramp_time ) 
    return (double)curr_time / (double)ramp_time;
  else if (curr_time < max_time - ramp_time)
    return 1.0;
  else if (curr_time < max_time)
    return ((double)(max_time - curr_time)) / (double)ramp_time;
  
  return 0;
  
}
