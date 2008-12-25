#include <string.h>
#include <math.h>

#include "controller.h"
#include "trajectory.h"
#include "common_def.h"
#include "traj_camera.h"

/* CHANGE T_STRAIGT TO DESIRED TRAJ INDEX LISTED IN trajectory.h
   in the enum */
#define THIS_TRAJ T_CAMERA


extern trajectory traj_array[NUM_TRAJ];

static void start(robo_state *rs, bot_params *bp);
static int run(robo_state *rs, robo_target *rt);
static void stop(void);

void init_camera(){
  traj_array[THIS_TRAJ].start = start;
  traj_array[THIS_TRAJ].run = run;
  traj_array[THIS_TRAJ].stop = stop;
}

/********************* FILL IN FUNCTIONS FOR YOUR ****************
 ********************* SPECIFIC TRAJECTORY   ********************/
static int time = 0;
static robo_state initial;
static straight_params params;
static int stopped = 1;

void start(robo_state * rs, bot_params * bp)
{
  int heading, diff;
  if(stopped)
    {

      /*
       * right from left
       150 to 180 
       current - (old-new) = current + 30
       
       left from right
       180 to 150
       current - (old-new) = current - 30
       
      */

      memcpy((void *)&initial,(void *)rs,sizeof(robo_state));
      memcpy((void *)&params,(void *)&(bp->straight),sizeof(straight_params));

      // Robot_Work(); // commented out- broken

      time = 0;
      stopped = 0;
    }

}
int run(robo_state *rs, robo_target *rt)
{
#define HEAD_P_GAIN(a) (a / 10)

  if(stopped)
    return TRAJ_DONE;

  rt->vel_L =  rt->vel_R = params.velocity * ramp_coefficient(time,params.time,500);
  rt->vel_R += HEAD_P_GAIN(rs->cam_pan_angle);
  rt->vel_L -= HEAD_P_GAIN(rs->cam_pan_angle);

  time += 26;


  if(time>=params.time)
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