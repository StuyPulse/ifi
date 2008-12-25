#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "user_routines.h"

#define MAX_POSITIONS 4

#define POS1 0
#define POS2 1
#define POS3 2
#define POS4 3
#define POS5 4
#define POS6 5

void Save_Arm_Pos(int pos_num, int pot_value);
int Arm_Pos(int pos_num);
void Load_Positions();
void User_Arm_Control(robo_state * rs, robo_target * rt, robo_out * ro);
void Set_Arm_Pos(robo_state *rs, robo_target *rt, robo_out *ro);

#endif
