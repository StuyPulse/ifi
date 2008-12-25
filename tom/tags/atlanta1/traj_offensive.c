#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "traj_offensive.h"
#include "user_routines.h"

#define THIS_TRAJ T_OFFENSIVE


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);




void init_offensive()
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

  if(step!=laststep)
    {
      switch(step)
	{
	case 0:
	  traj=T_TURN;
	  rt->grabber = UP;
	  bp.turn.deadzone = 5;
	  bp.turn.dir = 'c';
	  break;
	case 1:
	  traj=T_STRAIGHT_NOGYRO;
	  bp.straight.velocity = 127; //Full speed
	  bp.straight.heading = 0;
	  bp.straight.time = 0;
	  bp.straight.gear = MIDDLE;
	  bp.straight.distance = Target_Distance();
	  break;
	case 2:
	  traj=T_NULL;
	  bp.time.val = 0;
	  rt->grabber = DOWN;
	  break;
	case 3:
	  traj=T_STRAIGHT_NOGYRO;
	  bp.straight.velocity = -126;
	  bp.straight.heading = 0;
	  bp.straight.time = 1000;
	  bp.straight.gear = MIDDLE;
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
