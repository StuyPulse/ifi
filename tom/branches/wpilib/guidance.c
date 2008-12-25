#include "BuiltIns.h"
#include "OurDefines.h"
#include "guidance.h"

void get_state(struct robo_state *r)
{
  //long current_time_millis=GetMsClock();
  int time_elapsed = GetMsClock() - r->time_last_checked;
  int current_encoder_ticks;
  int current_x_accel;
  int current_y_accel;
  int direction;
  int velocity;
  
  if(r->gyro_enabled)
    r->heading = GetGyroAngle(r->rc_ports_s->gyro_p);

  if(r->accel_enabled)
    {
      //May I inquire why this can't just be r->accel_Y = ...?
      current_y_accel = GetAcceleration(r->rc_ports_s->accel_y_p);
      r->accel_Y = current_y_accel;
      current_x_accel = GetAcceleration(r->rc_ports_s->accel_x_p);
      r->accel_X = current_x_accel;
    }
  else
    //Turn the next line into something more meaningful, kindly.  Above too, maybe?
    direction = ( r->out->drive_r != 0 ? ( r->out->drive_r > 0 ? 1 : -1 ) : 0);

  if(r->encoders_enabled)
    {
      velocity = (GetEncoder(r->rc_ports_s->encoder_r_p) * r->constants->wheel_radius)/r->time_last_checked;
      if(r->out->drive_r>0) //If we're driving right side forward
	r->vel_R+=velocity;
      else
	r->vel_R-=velocity;
      
      PresetEncoder(r->rc_ports_s->encoder_r_p,0);

      velocity = (GetEncoder(r->rc_ports_s->encoder_l_p) * r->constants->wheel_radius)/r->time_last_checked;
      if(r->out->drive_l>0) //If we're driving right side forward
	r->vel_L+=velocity;
      else
	r->vel_L-=velocity;
      
      PresetEncoder(r->rc_ports_s->encoder_l_p,0);
    }


 if(r->arm_pot_enabled)
   {
     r->arm_pos=GetAnalogInput(r->rc_ports_s->arm_pot_p);
   }
}    


void init_rs(struct robo_state *r,struct oi_ports *oi_map, struct rc_ports *rc_map, struct robo_constants *consts)
{
  r->oi_ports_s = oi_map;
  DEBUG("drive_joy_l: %d\n",oi_map->drive_joy_l);
  r->rc_ports_s = rc_map;
  r->constants = consts;
  r->gyro_enabled=0;
  r->accel_enabled=0;
  r->encoders_enabled=0;
  if(consts->four_motor_drive)
    FourWheelDrive(rc_map->drive_l1,rc_map->drive_l2,rc_map->drive_r1,rc_map->drive_r2);
  else
    TwoWheelDrive(rc_map->drive_l1,rc_map->drive_r1);
  if(consts->invert_drive_l)
    {
      SetInvertedMotor(rc_map->drive_l1);
      SetInvertedMotor(rc_map->drive_l2);
    }
  if(consts->invert_drive_r)
    {
      SetInvertedMotor(rc_map->drive_r1);
      SetInvertedMotor(rc_map->drive_r2);
    }
	
}

void map_oi(struct robo_state *r)
{
  char r_inverted = r->constants->invert_drive_r;
  char l_inverted = r->constants->invert_drive_l;

  r->out->drive_l=GetOIAInput(r->oi_ports_s->drive_joy_l,Y_AXIS);
  r->out->drive_r=GetOIAInput(r->oi_ports_s->drive_joy_r,Y_AXIS);
  //DEBUG("drive_l_joy: %d\n",r->oi_ports_s->drive_joy_l);
  //DEBUG("drive_l: %d\n",r->out->drive_l);
  //DEBUG("drive_r: %d\n",r->out->drive_r);
  
  Motors(r->out->drive_l,r->out->drive_r);
  Motor(r->rc_ports_s->arm_p,r->out->arm);
}
