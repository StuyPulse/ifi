#include <string.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "traj_mckee.h"
#include "user_routines.h"

#define THIS_TRAJ T_MCKEE


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);




void init_mckee()
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
static char dir;

void start(robo_state * rs, bot_params * bp){
  if(stopped){
    memcpy((void *)&initial,(void *)rs,sizeof(robo_state));

    time = 0;
    stopped = 0;

    laststep = -1;
    step = 0;
    traj = T_NULL;

    dir = bp->charval.val;
  }
}

int run(robo_state *rs, robo_target *rt)
{

  bot_params bp;

  if(stopped)
    return TRAJ_DONE;

  printf("in mckee\r");
  
  if(step!=laststep)
    {
      switch(step)
	{
	case 0:
	  traj=T_STRAIGHT_NOGYRO;
	  bp.straight.gear = HIGH;
	  bp.straight.velocity = 120;
	  bp.straight.distance = 40*12;
	  bp.straight.time = 0;
	  bp.straight.heading = 0;
	  break;
	case 1:
	  traj=T_TURN;
	  bp.turn.speed = 127; //Full speed
	  bp.turn.time = 500;
	  bp.turn.dir = dir;
	  break;
	case 2:
	  traj=T_STRAIGHT_NOGYRO;
	  bp.straight.velocity = 127;
	  bp.straight.gear = LOW;
	  bp.straight.heading = 0;
	  bp.straight.time = 5000;
	default:
	  printf("Done\r\n");
	  return TRAJ_DONE;
	  break;
	}
      traj_array[traj].start(rs, &bp);
      
      laststep=step;
    }
  //  printf("traj# %d start %x run %x stop %x\r",traj,traj_array[traj].start,traj_array[traj].run,traj_array[traj].stop);

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
