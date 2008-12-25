#ifndef _TRAJ_STRAIGHT_NOGYRO_H
#define _TRAJ_STRAIGHT_NOGYRO_H

#define DS_TIME_T 0
#define DS_DISTANCE_T 1

#define GEAR_DELAY_MS 500
#define GEAR_SLOW_PD 500

typedef struct{
  float velocity;
  float distance;
  char gear;
  int heading;
  int time;
}straight_params;

void init_straight_nogyro(void);

#endif
