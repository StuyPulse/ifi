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

void init_controller(struct pidstate *s,int p_num, int p_den, int i_num, int i_den, int d_num, int d_den, int u_lim, int l_lim)
{
  if(p_den==0)
    {
      p_num=0;
      p_den=1;
    }
  if(d_den==0)
    {
      d_num=0;
      d_den=1;
    }
  if(i_den==0)
    {
      i_num=0;
      i_den=1;
    }
  s->p_gain_num = p_num;
  s->p_gain_den = p_den;
  s->i_gain_num = i_num;
  s->i_gain_den = i_den;
  s->d_gain_num = d_num;
  s->d_gain_den = d_den;
  s->u_lim = u_lim;
  s->l_lim = l_lim;
  s->error_sum = INITIAL_VALUE;
  s->last_error = 0;
}

void init_controller_divisor(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim)
{
  init_controller(s,1,p,1,i,1,d,u_lim,l_lim);
}

void init_controller_integer(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim)
{
  init_controller(s,p,1,i,1,d,1,u_lim,l_lim);
}

int controller(struct pidstate *s, int actual, int desired)
{
  int out=0;
  int error = desired - actual;
    
  //printf("Error: %d\r\n",error);

  //If error_sum is 0, then we've just started and last_error should
  //be set to the current error to avoid having a D term
  if(s->error_sum == INITIAL_VALUE)
    {
      s->last_error = error;
      s->error_sum = 0;
    }
  
  if(s->error_sum > WINDUP || s->error_sum < -WINDUP)
    s->error_sum=0;

  out += error * s->p_gain_num / s->p_gain_den;
  out += s->error_sum * s->i_gain_num / s->i_gain_den;
  out += (error - s->last_error) * s->d_gain_num / s->d_gain_den;

  if(out > s->u_lim)
    out = s->u_lim;
  if(out < s->l_lim)
    out = s->l_lim;

  s->last_error=error;
  s->error_sum+=error;

  return out;
}

float pd_vel_control(int side, float last_out, float desired, float actual)
{
 
  //long error;
  float error ;
  static float last_error_L = 0, last_error_R = 0;
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
  
/*	  if(out > 255)
	  out = 255;
	  if(out < 127)
	  out = 127;
*/
  
  // printf("get_velocity(): %ld.   get_rpm_1k(): %ld error: %ld P_GAIN(E): %d last_out: %d\r",get_velocity(1),get_rpm_1k(1)/1000,error,(int)P_GAIN(error),(int)last_out);
  return out;
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
