#include "trajectory.h"
trajectory traj_array[NUM_TRAJ];

void init_traj()
{
	init_null();
	init_straight();
	init_ram();
	init_turn_radius();
}

