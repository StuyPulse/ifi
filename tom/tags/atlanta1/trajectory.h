#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "user_routines.h"

/** INCLUDE ALL TRAJECTORY HEADERS HERE **/

#include "traj_null.h"
//#include "traj_turn_radius.h"
#include "traj_offensive.h"
#include "traj_straight_nogyro.h"
#include "traj_turn.h"

#define TRAJ_DONE 0
#define TRAJ_RUNNING 1

enum {
	T_NULL=0,
	//T_STRAIGHT,
	//T_TURN_RADIUS,
	//T_UTURN_AUTO,
	T_TURN,
	T_OFFENSIVE,
	T_STRAIGHT_NOGYRO,
	T_MCKEE,
	NUM_TRAJ
};

//#define T_AUTONOMOUS T_UTURN_AUTO

typedef struct 
{
  int val;
} int_params;

typedef struct
{
  char val;
} char_params;

#define time_params int_params

typedef union
{
  int_params intval;
  char_params charval;
  time_params time;
  straight_params straight;
  //  turn_radius_params turn_radius;
  turn_params turn;
} bot_params;

typedef struct 
{
  void (*start)(robo_state *rs, bot_params *bp);
  int (*run)(robo_state *rs, robo_target *rt);
  void (*stop)(void);
} trajectory;

//Add other init_<trajname> functions inside init_traj.
void init_traj(void);

#endif
