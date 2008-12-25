#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "user_routines.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_TURN

extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_turn(){
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static turn_params params;
static int stopped = 1;
static struct pidstate s;
	
void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      memcpy((void *)&params,(void *)&(bp->turn),sizeof(turn_params));
      init_controller(&s,1,5,1,135,0,1,127,-127);
      time = 0;      
      stopped = 0;
    }
}

int run(robo_state *rs, robo_target *rt)
{
  int error;
  int turnspeed;
  int pan_error;

  if(stopped)
    return TRAJ_DONE;
  
  printf("Running T_TURN\r\n");

  rt->gear = LOW;
  
  if(params.dir == 'l' || params.dir == 'L')
    {
      rt->vel_L = -params.speed;
      rt->vel_R = params.speed;
    }
  if(params.dir == 'r' || params.dir == 'R')
    {
      rt->vel_L = params.speed;
      rt->vel_R = -params.speed;
    }
  /*  if(params.dir == 'c')
    {
      pan_error = (int)T_Packet_Data.mx - PAN_TARGET_PIXEL_DEFAULT;
  
      
      if(T_Packet_Data.my != 0 && pan_error <= PAN_ALLOWABLE_ERROR_DEFAULT)
	{
	  error = rs->cam_pan_servo - PAN_CENTER_PWM_DEFAULT;
	  turnspeed=controller(&s,error,0);
	  rt->vel_L = -turnspeed;
	  rt->vel_R = turnspeed;
	}
      
      if(error<=params.deadzone || error >= -params.deadzone)
	{
	  return TRAJ_DONE;
	}
    }
  */
   if(params.dir != 'c' && time >= params.time)
    {
      return TRAJ_DONE;
    }

   time += 26;

  return TRAJ_RUNNING; 
}

void stop()
{
  stopped = 1;
}
