#include <string.h>
#include <math.h>
#include <stdio.h>

#include "controller.h"
#include "trajectory.h"
#include "common_def.h"
#include "traj_straight_nogyro.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */

#define THIS_TRAJ T_STRAIGHT_NOGYRO

extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_straight_nogyro(){
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static straight_params params;
static int gear_delay=0;
static int gear_slow = 0;
static int stopped = 1;
//static struct pidstate ps;

void start(robo_state * rs, bot_params * bp)
{

  int heading, diff;
  if(stopped)
    {

      memcpy((void *)&params,(void *)&(bp->straight),sizeof(straight_params));
#define NEAR_DISTANCE (params.velocity * .5)
      //init_controller(&ps,1000,NEAR_DISTANCE,0,1,0,1,127,-127);

      if(initial.heading >= 0)
	{
	  heading = initial.heading % (int)(PI*2.0*1000.0);
	  if(heading < 0)
	    heading += PI*2*1000;
	  
	  diff = heading-params.heading;

	  params.heading = rs->heading - diff;
	  /* I really want this stupid robot to work */
 	  //Robot_Work();
	}
      if(initial.gear != rs->gear)
	{
	  gear_delay = GEAR_DELAY_MS;
	  gear_slow = GEAR_SLOW_PD; 
	}
      time = 0;
      stopped = 0;
    }

}
  int run(robo_state *rs, robo_target *rt)
  {
    //distance from target
    //One of our encoders is broken, I forget which one.
    float current_distance = params.distance 
      - abs(rs->pos_L - initial.pos_L + rs->pos_R - initial.pos_R);
    
    float vel_change;

#define HEAD_P_GAIN(a) (a / 10.0)
#define DIST_P_GAIN(a) (1 - (NEAR_DISTANCE - a)/NEAR_DISTANCE)

    if(stopped)
      return TRAJ_DONE;

    printf("Running T_STRAIGHT\r\n");

    rt->gear = params.gear;

    if(time > gear_delay && time <= gear_delay + gear_slow)
      {
	rt->vel_L = rt->vel_R = 0;
	time += 26;
	return TRAJ_RUNNING;
      }
    if(params.time != 0)
      rt->vel_L =  rt->vel_R = 
	params.velocity * ramp_coefficient(time,params.time,500);

    if(params.heading >= 0)
      {/*       
	if(params.heading == 0)
	  {
	    vel_change = HEAD_P_GAIN(rs->pos_L - rs->pos_R);
	    rt->vel_R += vel_change;
	    rt->vel_L -= vel_change;
	  }
	else
	  {
	    printf("no gyro, we're fuuucked!\r\n");
	  }
       */
      }
    else //Use camera
      {
	rt->vel_R += HEAD_P_GAIN(rs->cam_pan_angle);
	rt->vel_L -= HEAD_P_GAIN(rs->cam_pan_angle);
      }


    if(params.distance != 0)
      {
	//vel_change = controller(&ps,current_distance,0) / 1000.0;
	rt->vel_R = params.velocity;
	rt->vel_L = params.velocity;
	printf("posL: %d\tposR: %d",(int)rs->pos_L,(int)rs->pos_R);
	if(current_distance <= 0)
	  {
	    rt->vel_L = rt->vel_R = 0;

	    return TRAJ_DONE;
	  }
      }
    

  
    time += 26;


    if((params.time != 0 && time>=params.time))
      {
	rt->vel_L = rt->vel_R = 0;
	return TRAJ_DONE;
      }

    return TRAJ_RUNNING; 
  
  }


  void stop()
  {
    stopped = 1;
  }
