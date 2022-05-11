#include <stdio.h>

#include "velocity.h"
#include "common_def.h"


extern volatile unsigned int left_clock;
extern volatile unsigned int right_clock;
extern volatile unsigned int fly_clock;
extern volatile int left_vel;
extern volatile int right_vel;
extern volatile int fly_vel;
int left_vel_arr[SAMPLES_AVERAGED_DRIVE];
int right_vel_arr[SAMPLES_AVERAGED_DRIVE];
int fly_vel_arr[SAMPLES_AVERAGED_FLYWHEEL];
unsigned int left_index;
unsigned int right_index;
unsigned int fly_index;


long get_rpm_1k(int side){
#if 0
  if(side == FLY_SENSOR){
    long rpm_1k = 0;

    /*count / spokes = Num_revolutions
      Num_revolutions / clock = rev/ms
      rev/ms * 1000 * 60 * 1000 = rpm_1k
    */

    if(fly_clock != 0)
      rpm_1k = 60000000L * fly_count / NUM_FLY_SPOKE / fly_clock;

    fly_clock = fly_count = 0;
    return rpm_1k;

  }
  else
#else
    {
      long ms = get_velocity(side);
      int spokes = side==FLY_SENSOR?NUM_FLY_SPOKE:NUM_DRIVE_SPOKE;
      if(ms <= 0)
	{
	  switch(ms)
	    {
	    case 0:
	      printf("ms 0 wtf\r");
	      break;
	    case -1:
	      printf("Illegal side\r");
	      break;
	    case -2:
	      printf("Sum zero\r");
	      break;
	    case -3:
	      printf("Count zero\r");
	      break;
	    default:
	      break;
	    }
	  return 0;
	}
      return 60000000L / (spokes*ms);
    }
#endif
}

long get_velocity(int side)
{
  int *vel;
  int last_vel;
  unsigned int clock;
  unsigned *index;
  int i,error,sa,fluc;
  long vel_avg=0, vel_count=0;
  if(side == RIGHT_SIDE){
    vel = right_vel_arr;
    last_vel = right_vel;
    clock = right_clock;
    index = &right_index;
    sa = SAMPLES_AVERAGED_DRIVE;
    fluc = FLUCTUATION_DRIVE;
  }
  else if(side == LEFT_SIDE){
    vel = left_vel_arr;
    last_vel = left_vel;
    clock = left_clock;
    index = &left_index;
    sa = SAMPLES_AVERAGED_DRIVE;
    fluc = FLUCTUATION_DRIVE;
  }
  else if(side == FLY_SENSOR){
    vel = fly_vel_arr;
    last_vel = fly_vel;
    clock = fly_clock;
    index = &fly_index;
    sa = SAMPLES_AVERAGED_FLYWHEEL;
    fluc = FLUCTUATION_FLYWHEEL;
  }
  else{
    return -1;
  }
  vel[*index]=last_vel;
  for(i=0;i<sa;i++)
    {
      error = vel[i]-vel[*index];
      if(error >= -fluc && error <= fluc )
	{
	  //printf(" %d: %d\n",i,vel[i]);
	  vel_avg += vel[i];
	  vel_count++;
	}
    }
  *index++;
  if(!vel_avg)
    return -2;
  if(vel_count)
    {
      if(vel_avg+fluc < clock)
	return clock;
      return vel_avg/vel_count;
    }
  return -3;
}


#if 0 
int old_get_velocity(int side)
{
  int *vel;
  unsigned int *clock;
  long vel_avg=0, vel_sum=0;
  int i;
  
  vel = side ? left_vel : right_vel;
  clock = side ? &left_clock : &right_clock;
  
  // weighted average of all pulse times
  for(i=0;i<SAMPLES_AVERAGED_DRIVE;i++)
    {
      vel_avg += vel[i] * vel[i];
      vel_sum += vel[i];
    }
  // add in the current count, but do not add to the sum
  vel_avg += *clock * *clock ;
  
  if ( vel_sum == 0 )
    return -1 ;
  
  vel_avg /= vel_sum;
  
  if ( *clock > 30000 )
    return -1 ;
  
  return vel_avg ;
}
/*
  int left_velocity(void)
  {
  long left_vel_avg=0, left_vel_sum=0;
  int i;
  
  // weighted average of all pulse times
  for(i=0;i<SAMPLES_AVERAGED_LEFT;i++)
  {
  left_vel_avg += left_vel[i] * left_vel[i];
  left_vel_sum += left_vel[i];
  }
  // add in the current count, but do not add to the sum
  left_vel_avg += left_clock * left_clock ;
  
  if ( left_vel_sum == 0 )
  return -1 ;

  left_val_avg /= left_vel_sum;

  if ( left_clock > 60000 )
  {
  for ( i = 0 ; i < SAMPLES_AVERAGED_LEFT ; i++ ) left_vel[i] = 0 ;
  left_clock = 0 ;
  return -1 ;
  }
  //if(left_clock > left_val_avg)
  //	return left_clock;

  return left_vel_avg ;
  }

  int right_velocity(void)
  {
  int right_vel_avg;
  int i;

  for(i=0;i<SAMPLES_AVERAGED_RIGHT;i++)
  {
  right_vel_avg += right_vel[i];
  }
  }
*/
#endif