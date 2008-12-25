#include "trajectory.h"
trajectory traj_array[NUM_TRAJ];

void init_traj()
{
	init_straight_nogyro();
	init_offensive();
	init_null();
	init_turn();
	init_mckee();
}

