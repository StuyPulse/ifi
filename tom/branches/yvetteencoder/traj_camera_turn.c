#include <string.h>
#include "controller.h"
#include "trajectory.h"
#include "user_routines.h"
#include "camera.h"
#include "tracking.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_CAMERA_TURN

extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_camera_turn(){
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static int deadzone;
static int stopped = 1;
static struct pidstate s;
	
void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      deadzone = bp->camera.deadzone;
      init_controller(&s,1,5,1,135,0,1,48,-48);
      stopped = 0;
    }
}

int run(robo_state *rs, robo_target *rt)
{
  int error;
  int turnspeed;

  if(stopped)
    return TRAJ_DONE;


  rt->gear = LOW;
  
  if(T_Packet_Data.my != 0)
    {
      error = rs->cam_pan_servo - PAN_CENTER_PWM_DEFAULT;
      turnspeed=controller(&s,error,0);
      rt->vel_L = -turnspeed;
      rt->vel_R = turnspeed;
    }

  if(error<=deadzone || error >= -deadzone)
    {
      return TRAJ_DONE;
    }

  return TRAJ_RUNNING; 
}

void stop()
{
  stopped = 1;
}
