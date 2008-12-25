#include <string.h>
#include "controller.h"
#include "trajectory.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_SCORE

extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_score(){
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
static int laststep=-1;
static int step=0;
static int traj=T_NULL;


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
  bot_params bp;
  
  if(stopped)
    return TRAJ_DONE;


  
  if(step!=laststep)
    {
      switch(step)
	{
	case 0:
	  traj=T_STRAIGHT;
	  bp.straight.velocity = 90; //7.5 feet/second
	  bp.straight.heading = -1;
	  bp.straight.time = 25/7.5+2 + 1; //Going a second too far.
	  break;
	default:
	  return TRAJ_DONE;
	  break;
	}
        traj_array[traj].start(rs, &bp);
      
      laststep=step;
    }
  printf("traj# %d start %x run %x stop %x\r",traj,traj_array[traj].start,traj_array[traj].run,traj_array[traj].stop);
  if(traj_array[traj].run(rs, rt) == TRAJ_DONE)
    {
      traj_array[traj].stop();
      step++;
    }

  return TRAJ_RUNNING; 
}

void stop()
{
  stopped = 1;
}
