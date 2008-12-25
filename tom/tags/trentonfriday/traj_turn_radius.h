#ifndef _TRAJ_TURN_RADIUS_H
#define _TRAJ_TURN_RADIUS_H

typedef struct{
  float radius;
  int heading;
  float velocity;
  int time;
  int dead_zone;
} turn_radius_params;

void init_turn_radius(void);

#endif
