#include <stdio.h>
#include "ir.h"
#include "controller.h"
#include "user_routines.h"
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "gyro.h"
#include "common_def.h"
#include "arm.h"

char ir;
static struct pidstate pid_str, pid_turn, pid_str2;

char check_ir(){
  static int count = 0;
  static char res = 0;
  //printf("count %d ir ins: %d %d %d %d\n", count, IR1, IR2, IR3, IR4);

  if(!(IR1 || IR2 || IR3 || IR4)) //no buttons pressed?
    if(count < 6)
      count++;
    else //been too many loops since got a high input
       res = 0;
  else{ //button pressed!
    count = 0;
    //only one button pressed at a time
    res = IR1 + IR2 * 2 + IR3 * 3 + IR4 * 4;
  }
  return res;
}

void init_ir(){
  //for going straight
  init_controller(&pid_str,  1, 8,    0, 250,   2, 1,   50, -50);
  //for coming out of the 45-degree angle turn
  init_controller(&pid_str2, 1, 30,   0, 250,   2, 5,   17, -17);
  //for doing the turns
  init_controller(&pid_turn, 1, 15,   0, 1,     2, 5,   50, -50);
}

//meant to be called once in each loop through the autonomous 
void IR_Drive(){
  int ang = Get_Gyro_Angle();
  char c = 0;
  static int ang0;
  static char state = -1;
  //static int lastir = -1;
  static int t = 0;
  static char armstate = 0;

  t++;

  if(state == -1){
    ang0 = ang;
    state = 0;
    if(CORNER_SW_MID){
      state = 6;
      ang0 -= 785;
    }
  }

  switch(ir){
  case 1: //up to turn right angle left
    state = 3;
    break;
  case 2: //down: poke
    t = 0;
    state = 4;
    break;
  case 3: //left: drift left
    if(state != 3) //don't go if turning
      state = 1;
    break;
  case 4: //right: drift right
    if(state != 3) //don't go if turning
      state = 2;
    break;
  case 0: //nothing: go straight if currently drifting or going straight
    if(state <= 2)
      state = 0;
    break;
  }

  /*
    state:
    6: going straight from corner position (lasts ~2 seconds)
    0: going straight
    1: heading left
    2: heading right
    3: turning
    4: poke up
    5: poke down
  */
  switch(state){
  case 6: //going straight from corner
    c = (char)controller(&pid_str2, ang, ang0);
    if(t >= 76) //2 seconds
      state = 0;
    break;
  case 0: //straight
    c = (char)controller(&pid_str, ang, ang0);
    break;
  case 1: //drift left
    c = (char)controller(&pid_turn, ang, ang0 + DRIFT_ANG);
    break;
  case 2: //drift right
    c = (char)controller(&pid_turn, ang, ang0 - DRIFT_ANG);
    break;
  case 3: //turn left
    c = (char)controller(&pid_turn, ang, ang + 600);
    if(ang >= ang0 + 1570 * 19 / 20){ //exit turn early: don't overshoot
      ang0 += 1570;
      state = 0;
    }
    break;

  case 4: //stop moving, raise arm
    if(t < 10){ //drive a bit backwards for a short time
      R_DRIVE1 = R_DRIVE2 = 127 + 10;
      L_DRIVE1 = L_DRIVE2 = 127 - 10;
    }
    else
      R_DRIVE1 = R_DRIVE2 = L_DRIVE1 = L_DRIVE2 = 127;
    armstate = 1;
    break;
  case 5: //lower arm after poke
    R_DRIVE1 = R_DRIVE2 = L_DRIVE1 = L_DRIVE2 = 127;
    armstate = 2;
    break;
  }

  /*
    0: move back to low poking position in the beginning
    1: raise arm to poke
    2: lower arm after poke
    3: do nothing
  */
  switch(armstate){
  case 0: //going to low poking
    //move arm up
    TILT_F = 1;
    TILT_R = 0;
    ARM = 0;
    if(Get_ADC_Result(ARM_POT) < ARM_POKE_POS_LOW)
      ARM = 127;
    break;
  case 1: //raise to poke
    ARM = 255;
    if(Get_ADC_Result(ARM_POT) > ARM_POKE_POS_HIGH){
      ARM = 127;
      state = 5; //this causes armstate to be set
    }
    break;
  case 2: //lower after poke
    ARM = 0;
    if(Get_ADC_Result(ARM_POT) < ARM_POKE_POS_LOW){
      ARM = 127;
      armstate = 3;
      //return to going straight
      state = 0;
    }
    break;
  case 3:
    ARM = 127;
    break;
  }

  printf("ang %d ang0 %d irstate %d c %d state %d armstate %d\n", ang, ang0, ir, c, armstate, state);

  if(state <= 3){ // only do this if drifting or turning
    L_DRIVE1 = L_DRIVE2 = 127 + 40 - c;
    R_DRIVE1 = R_DRIVE2 = 127 - 40 - c;
  }
}
