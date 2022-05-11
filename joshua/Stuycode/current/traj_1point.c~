#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "traj_1point.h"

#define THIS_TRAJ T_1POINT


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_1point(){
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

  if(step!=laststep)
    {
      switch(step)
	{
	case 0:
	  traj=T_STRAIGHT;
	  bp.straight.velocity = 72; // 6ft/sec
	  bp.straight.heading = 0; 
	  bp.straight.time = 2500;
	  break;
	case 1:
	  traj=T_SPIRAL;
	  bp.intval.val = REVERSE;
	  break;
	case 2:
	  traj=T_ACQUISITOR;
	  bp.intval.val = REVERSE;
	  break;
	default:
	  return TRAJ_DONE;
	  break;
	}
        traj_array[traj].start(rs, &bp);
      
      laststep=step;
    }
  //    printf("traj# %d start %x run %x stop %x\r",traj,traj_array[traj].start,traj_array[traj].run,traj_array[traj].stop);
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
