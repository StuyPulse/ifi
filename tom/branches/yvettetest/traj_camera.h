#ifndef _TRAJ_STRAIGHT_H
#define _TRAJ_STRAIGHT_H

#define DS_TIME_T 0
#define DS_DISTANCE_T 1

struct camera_params {
  float velocity;
  int side;
  int distance;
};

void init_camera(void);

#endif
