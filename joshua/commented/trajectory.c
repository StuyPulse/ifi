#include "trajectory.h"
trajectory traj_array[NUM_TRAJ];

void init_traj()
{
	init_null();
	init_straight();
	init_turn_radius();
	init_shooter();
	init_spiral();
	init_acquisitor();
	//	init_uturn_auto();
	init_turnshoot();
	init_1point();
	//	init_ram();
	//init_straightshoot();
}

