#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "eeprom.h"
#include "user_routines.h"
#include "arm_control.h"
#include "controller.h"

unsigned int positions[MAX_POSITIONS];

void Save_Arm_Pos(int pos_num, int pot_value)
{
  positions[pos_num] = pot_value;
  EEPROM_Write_Int(pos_num,pot_value);
}

int Arm_Pos(int pos_num)
{
  return positions[pos_num];
}

void Load_Positions(void)
{
  int i;

  for(i=0; i < MAX_POSITIONS; i++)
    positions[i] = EEPROM_Read_Int(i*BLOCKSIZE);
}

void User_Arm_Control(robo_state *rs, robo_target *rt, robo_out *ro)
{
  static int already_pressed = 0;
  if(ARM_SAVE_MODE)
    {
      Relay2_green = 1;
      rt->arm_pos=-1;
      if(POS1_SW)
	Save_Arm_Pos(0,rs->arm_pos);
      else if(POS2_SW)
	Save_Arm_Pos(1,rs->arm_pos);
      else if(POS3_SW)
	Save_Arm_Pos(2,rs->arm_pos);
      else if(POS4_SW)
	Save_Arm_Pos(3,rs->arm_pos);
      else if(POS5_SW)
	Save_Arm_Pos(3,rs->arm_pos);
      else if(POS6_SW)
	Save_Arm_Pos(3,rs->arm_pos);

    }
  else
    {
      Relay2_green = 0;
      if(POS1_SW)
	rt->arm_pos = Arm_Pos(POS1);
      else if(POS2_SW)
	rt->arm_pos = Arm_Pos(POS2);
      else if(POS3_SW)
	rt->arm_pos = Arm_Pos(POS3);
      else if(POS4_SW)
	rt->arm_pos = Arm_Pos(POS4);
      else if(POS5_SW)
	rt->arm_pos = Arm_Pos(POS5);
      else if(POS6_SW)
	rt->arm_pos = Arm_Pos(POS6);
      else
	rt->arm_pos = -1;
      
    }
  if(rt->arm_pos < 0)
    ro->arm = ARM_JOY;
  else
    Set_Arm_Pos(rs,rt,ro);
  
}

/*******************************************************************************
 * FUNCTION NAME: Set_Arm_Pos
 * PURPOSE:       Calculates a speed to set the arm motor to using a PID controller.
                  The speed is stored in the robo_out struct ro.  
 * CALLED FROM:   control
 * ARGUMENTS:     a robo_state* with the current state of the robot, a robo_target*
                  with the desired arm position, and a robo_out* to store the motor
                  speed in
 * RETURNS:       void
 *******************************************************************************/
void Set_Arm_Pos(robo_state *rs, robo_target *rt, robo_out *ro)
{
  static int running=0;
  static int neartop=0;
  static struct pidstate ps;
  static int old_arm_pos=-1;
  if(!running || old_arm_pos != rt->arm_pos)
    {
      init_controller(&ps,ARM_P_GAIN_N,ARM_P_GAIN_D,ARM_I_GAIN_N,ARM_I_GAIN_D,0,0,127,-127);
      old_arm_pos = rt->arm_pos;
      running=1;
    }
  /* if(INRANGE(rs->arm_pos,ARM_TOP,240) && INRANGE(rt->arm_pos,ARM_TOP,50))// && !neartop)
    {
      ps.p_gain_num = 2 * ARM_P_GAIN_N;
      ps.p_gain_den = 3 * ARM_P_GAIN_D;
      ps.i_gain_num = 0;
      neartop = 1;
      /*
      if(INRANGE(rs->arm_pos,rt->arm_pos,40))
	{
	  ro->arm = 127;
	  return;
	}
    
    }
  if(INRANGE(rs->arm_pos,ARM_TOP,130) && INRANGE(rt->arm_pos,ARM_TOP,50))// && !neartop)
    {
      ps.p_gain_num = 3 * ARM_P_GAIN_N;
      ps.p_gain_den = 7 * ARM_P_GAIN_D;
      ps.i_gain_num = 0;
      neartop = 1;
    }
  else
    {
      ps.p_gain_den = ARM_P_GAIN_D;
      ps.i_gain_num = ARM_I_GAIN_N;
      }
  */
  ro->arm = 127 + controller(&ps,rs->arm_pos,rt->arm_pos);
  /*
  printf("p_gain_den: %d\r\n",ps.p_gain_den);
  printf("ro->arm: %d\r\n",ro->arm);
  printf("rt->arm_pos: %d\r\n",rt->arm_pos);
  printf("rs->arm_pos: %d\r\n",rs->arm_pos);
  */
 
}
