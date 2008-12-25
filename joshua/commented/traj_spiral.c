#include <string.h>

#include "controller.h"
#include "trajectory.h"
#include "traj_spiral.h"

#define THIS_TRAJ T_SPIRAL


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_spiral()
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
static int direction;

void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      direction = bp->intval.val;
      time = 0;
      stopped = 0;
    }
}

int run(robo_state *rs, robo_target *rt)
{
  //printf("in traj spiral\r") ;
  if(stopped)
    return TRAJ_DONE; 

  rt->spiral = direction;
  
  return TRAJ_DONE;
}


void stop()
{
  stopped = 1;
}
