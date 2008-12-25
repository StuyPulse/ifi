#ifndef CONTROLLER_H
#define CONTROLLER_H

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

int controller(struct pidstate *s, int actual, int desired);

#endif CONTROLLER_H
