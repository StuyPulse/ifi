#ifndef _TRAJ_STRAIGHT_H
#define _TRAJ_STRAIGHT_H

#define DS_TIME_T 0
#define DS_DISTANCE_T 1

typedef struct{
  float velocity;
  int side;
  int distance;
} camera_params;

void init_straight(void);

#endif
