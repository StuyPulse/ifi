#include "controller.h"

int controller(struct pidstate *s, int actual, int desired)
{
  int out=0;
  int error = desired - actual;

  //If error_sum is 0, then we've just started and last_error should
  //be set to the current error to avoid having a D term
  if(s->error_sum == 0)
    s->last_error = error;
  //Now now, what if the integral is magically 0?  Use a magic number that you initialize to, rather than 0.
  
  if(s->error_sum > 30000 || s->error_sum < -30000)
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
  
