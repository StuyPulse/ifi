#ifndef controll_H
#define controll_H

struct pidstate {
  int p_gain_num;
  int p_gain_den;

  int i_gain_num;
  int i_gain_den;

  int d_gain_num;
  int d_gain_den;

  int error_sum;
  int last_error;

  int u_lim;
  int l_lim;
};

#define P_GAIN_R(a)  (a * 1 / (33.0) )
#define D_GAIN_R(a,b)  ((a-b) / 4.0)

#define P_GAIN_L(a)  (a * 1 / (33.0) )
#define D_GAIN_L(a,b)  ((a-b) / 4.0)

#define WINDUP 30000
#define INITIAL_VALUE (WINDUP+1)

int controller(struct pidstate *s, int actual, int desired);
void init_controller(struct pidstate *s,int p_num, int p_den, int i_num, int i_den, int d_num, int d_den, int u_lim, int l_lim);
void init_controller_divisor(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim);
void init_controller_integer(struct pidstate *s,int p, int i, int d, int u_lim, int l_lim);
float pd_vel_control(int side, float last_out, float desired, float actual);
float ramp_coefficient(int curr_time, int max_time, int ramp_time);

#endif
