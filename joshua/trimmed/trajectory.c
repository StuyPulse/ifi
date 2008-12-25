#include "trajectory.h"
trajectory traj_array[NUM_TRAJ];

void init_traj()
{
	init_null();
	init_straight();
	init_turn_radius();
	init_uturn_auto();
	init_ram();
}

