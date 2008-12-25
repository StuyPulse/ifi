#ifndef _SCHEMES_H
#define _SCHEMES_H

#define OI_SCHEME &yvette_oi
#define RC_SCHEME &tom_rc
#define CONSTANTS &tom_constants

struct oi_ports
{
  //I'd suggest using _in_ instead of _joy_ since it could be controlled by other means.
  char drive_joy_l;
  char drive_joy_r;
  char arm_joy;
};

struct rc_ports
{
  //Outputs
  char drive_l1;
  char drive_l2;
  char drive_r1;
  char drive_r2;

  char arm_p; //p?

  //Digital/interrupt sensors
  char encoder_l_p;
  char encoder_r_p;
  
  //Analog sensors
  char gyro_p;
  char accel_x_p;
  char accel_y_p;
  char arm_pot_p;
};

struct robo_constants
{
  int wheel_base; //inches
  int wheel_radius; //.1 inches

  int four_motor_drive;

  char invert_drive_l;
  char invert_drive_r;
};

#ifndef _SCHEME_C

extern const struct oi_ports yvette_oi;
extern const struct rc_ports tom_rc;
extern const struct robo_constants tom_constants;
extern const struct rc_ports yvette_rc;
extern const struct robo_constants yvette_constants;

#endif

#endif
