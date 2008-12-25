#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
//#include "interrupts.h"
//#include "velocity.h"
//#include "common_def.h"
#include "controller.h"

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
/*
void init_controller_divisor(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim){
  init_controller(s,1,p,1,i,1,d,u_lim,l_lim);
}

void init_controller_integer(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim){
  init_controller(s,p,1,i,1,d,1,u_lim,l_lim);
}
*/

int controller(struct pidstate *s, int actual, int desired){
  int out=0;
  int error = desired - actual;
    
  //If error_sum is 0, then we've just started and last_error should
  //be set to the current error to avoid having a D term
  if(s->error_sum == INITIAL_VALUE)
    {
      //printf("resetting error\n");
      s->last_error = error;
      s->error_sum = 0;
    }
  
  if(s->error_sum > WINDUP || s->error_sum < -WINDUP)
    s->error_sum=0;

  out += error * s->p_gain_num / s->p_gain_den;
  out += s->error_sum * s->i_gain_num / s->i_gain_den;
  out += (error - s->last_error) * s->d_gain_num / s->d_gain_den;

  /*
  printf("p_gain %d/%d error %d\n", s->p_gain_num, s->p_gain_den, error);
  printf("p_change %d\n", error * s->p_gain_num / s->p_gain_den);
  printf("i_change %d\n", s->error_sum * s->i_gain_num / s->i_gain_den);
  printf("d_change %d\n", (error - s->last_error) * s->d_gain_num / s->d_gain_den);
  */

  if(out > s->u_lim)
    out = s->u_lim;
  if(out < s->l_lim)
    out = s->l_lim;

  s->last_error=error;
  s->error_sum+=error;


  //printf("Error: %d out %d\r\n",error, out);

  return out;
}

/*
float ramp_coefficient(int curr_time, int max_time, int ramp_time){
  if(curr_time < ramp_time)
    return (double)curr_time / (double)ramp_time;
  else if (curr_time < max_time - ramp_time)
    return 1.0;
  else if (curr_time < max_time)
    return ((double)(max_time - curr_time)) / (double)ramp_time;
  
  return 0;
  
}
*/
