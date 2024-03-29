#include <string.h>
#include "controller.h"
#include "trajectory.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_NULL

extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_null(){
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static int max_time;
static int stopped = 1;

void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      max_time = bp->time.val;
      time = 0;
      stopped = 0;
    }
}

int run(robo_state *rs, robo_target *rt)
{
  if(stopped)
    return TRAJ_DONE;

  rt->vel_L =  rt->vel_R = 0;

  time += 26;

  if(time>=max_time)
    {
      return TRAJ_DONE;
    }

  return TRAJ_RUNNING; 
}

void stop()
{
  stopped = 1;
}
