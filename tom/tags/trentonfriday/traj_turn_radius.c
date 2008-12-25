#include <string.h>
#include <math.h>
#include <stdio.h>
#include "controller.h"
#include "trajectory.h"
#include "common_def.h"
#include "traj_turn_radius.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_TURN_RADIUS


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_turn_radius()
{
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static turn_radius_params params;
static int stopped = 1;
static float vel_L,vel_R;

void start(robo_state * rs, bot_params * bp)
{
  if(stopped)
    {
      int heading,diff;
      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      memcpy((void *)&params,(void *)&(bp->turn_radius),sizeof(turn_radius_params));
      if(params.radius > -HALF_WHEEL_BASE && params.radius < HALF_WHEEL_BASE)
	{
	  if(params.radius < 0)
	    {
	      params.radius = -HALF_WHEEL_BASE;
	    }
	  else 
	    params.radius = HALF_WHEEL_BASE;
	}
      
      heading = initial.heading % (int)(PI*2.0*1000.0);
      if(heading < 0)
	heading += PI*2*1000;
      
      /*
       * right from left
       150 to 180 
       current - (old-new) = current + 30
       
       right from right
       210 to 180 
       current + 360-(old-new) = current + 330 

      
              
       left from right
       180 to 150
       current - (old-new) = current - 30
       
       left from left
       180 to 210
       current - (360+(old-new)) = current - 330
      
       */
      
      diff = heading - params.heading; //old - new
      if(params.radius > 0)
	{
	  if(params.heading > heading)
	    { //Right from left
	      params.heading = initial.heading - diff;
	    }
	  else
	    { //Right from right.
	      params.heading = initial.heading + 360 - diff;
	    }
	}
      else
	{
	  if(params.heading < heading)
	    { //Left from right
	      params.heading = initial.heading - diff;
	    }
	  else
	    { //Left from left
	      params.heading = initial.heading - (360+diff);
	    }
	}
      vel_L = vel_R = params.velocity;
      vel_L *= (params.radius+HALF_WHEEL_BASE)/params.radius;
      vel_R *= (params.radius-HALF_WHEEL_BASE)/params.radius;
      time = 0;
      stopped = 0;
    }
}
int run(robo_state *rs, robo_target *rt)
{

  //printf("In traj_turn_radius\n");
  if(stopped)
    return TRAJ_DONE;
  //printf("In traj_turn_radius 2\n");
  if((rs->heading > (params.heading - params.dead_zone)) &&
     (rs->heading < (params.heading + params.dead_zone)))
    return TRAJ_DONE;

  //printf("In traj_turn_radius 3\n");
  rt->vel_L = vel_L * ramp_coefficient(time,params.time,300);
  rt->vel_R = vel_R * ramp_coefficient(time,params.time,300);

  //printf("Vel L: %d Vel R: %d\n",(int)rt->vel_L, (int)rt->vel_R);
  time += 26;

  if(time>=params.time)
    {
      //printf("In traj_turn_radius 4 %d\n", time);
      rt->vel_L = rt->vel_R = 0;
      return TRAJ_DONE;
    }

  //printf("In traj_turn_radius 5 %d\n", time);
  return TRAJ_RUNNING; 
}


void stop()
{
  stopped = 1;
}
