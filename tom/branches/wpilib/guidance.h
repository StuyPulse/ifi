#ifndef _GUIDANCE_H
#define _GUIDANCE_H

#include "schemes.h"

struct robo_state {
  short gyro_enabled;
  short accel_enabled;
  short arm_pot_enabled;
  short encoders_enabled;
  
  int pos_X; //inches  * should be at least 0.1
  int pos_Y;

  int vel_L; //inches/s  * should be at least 0.1
  int vel_R;

  int accel_Y;
  int accel_X;
  
  int heading; //0.1 degrees

  int arm_pos;

  long time_last_checked;

  short reverse_drive; //Needs implementation!  Remove this when that's done.

  struct rc_ports *rc_ports_s;
  struct oi_ports *oi_ports_s;
  struct robo_constants *constants;
  struct robo_out *out;
};

// These values will be outputted, inverted if necessary using
// robo_constants AND robo_state.reverse_drive
struct robo_out
{
  char drive_l;
  char drive_r;
  char arm;
};


void init_rs(struct robo_state *r,struct oi_ports *oi_map, struct rc_ports *rc_map, struct robo_constants *consts);
void get_state(struct robo_state *r);
void map_oi(struct robo_state *r);

#endif
