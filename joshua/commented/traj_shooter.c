#include <string.h>

#include "controller.h"
#include "trajectory.h"
#include "traj_shooter.h"

/* CHANGE T_TEMPLATE TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_SHOOTER


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

/*CHANGE THE NAME OF THIS, AND ADD IT TO init_traj IN trajectory.c*/
void init_shooter()
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
static int direction = 0;

void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      direction = bp->intval.val;
      stopped = 0;
    }
}

int run(robo_state *rs, robo_target *rt)
{
  //printf("in traj shooter\r") ;
  if(!stopped)
    rt->shooter = direction;
  return TRAJ_DONE;
  
}


void stop()
{
  stopped = 1;
}
