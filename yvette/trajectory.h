#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "user_routines.h"

/** INCLUDE ALL TRAJECTORY HEADERS HERE **/

#include "traj_null.h"

#define TRAJ_DONE 0
#define TRAJ_RUNNING 1

enum {
	TRAJ_NULL,	
	NUM_TRAJ
};

//#define T_AUTONOMOUS T_UTURN_AUTO

typedef struct 
{
  int val;
} int_params;

#define time_params int_params

typedef union
{
  int_params intval;
  time_params time;
}bot_params;

typedef struct 
{
  void (*start)(robo_state *rs, bot_params *bp);
  int (*run)(robo_state *rs, robo_target *rt);
  void (*stop)(void);
}trajectory;

//Add other init_<trajname> functions inside init_traj.
void init_traj(void);

#endif
