#ifndef _TRAJ_TURN_H
#define _TRAJ_TURN_H

void init_turn(void);

typedef struct{
  int speed;
  int time;
  char dir;
  int deadzone;
}turn_params;


#endif
