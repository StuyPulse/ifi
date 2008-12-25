#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "traj_turnshoot.h"

/* CHANGE T_STRAIGHT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_TURNSHOOT


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);


void init_turnshoot()
{
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;

static int stopped = 1;
static int laststep=-1;
static int step=0;
static int traj=T_NULL;

void start(robo_state * rs, bot_params * bp){
  if(stopped){
    memcpy((void *)&initial,(void *)rs,sizeof(robo_state));

    time = 0;
    stopped = 0;

    laststep = -1;
    step = 0;
    traj = T_NULL;
  }
}

int run(robo_state *rs, robo_target *rt)
{
  bot_params bp;

  if(stopped)
    return TRAJ_DONE;

  rt->reverse_drive=1;
  if(step!=laststep)
    {
      switch(step)
	{
#if 0
	case 0:
	  traj=T_SHOOTER;
	  bp.intval.val=FORWARD;
	  break;
	case 1:
	  traj=T_STRAIGHT;
	  bp.straight.velocity=72;
	  bp.straight.heading=-1;//rs->cam_pan_angle;
	  bp.straight.time=2800;
	  break;
	case 2:
	  if( (T_Packet_Data.my == 0) || (rs->cam_pan_angle < -170) || (rs->cam_pan_angle > 170))
	    return TRAJ_DONE;
	  traj=T_SPIRAL;
	  bp.intval.val=FORWARD;
	  break;
	default:
	  return TRAJ_DONE;
	  break;
#endif
	}
      traj_array[traj].start(rs, &bp);
      
      laststep=step;
    }

  if(traj_array[traj].run(rs, rt) == TRAJ_DONE)
    {
      //printf("TRAJ_DONE \n");
      traj_array[traj].stop();
      step++;
    }
  return TRAJ_RUNNING;
}


void stop()
{
  stopped = 1;
}
