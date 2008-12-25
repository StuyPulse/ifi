#include <stdio.h>
#include "common_def.h"
#include "user_routines.h"
#include "arm.h"
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "controller.h"

struct pidstate armpid;

void init_arm(){
  init_controller(&armpid, 2, 1, 1, 2000, 1, 10, 125, -125);
}

void do_arm(){
  char c;
  int gotopos;
  int armpot = Get_ADC_Result(ARM_POT);
  static char armstate = ARM_MANUAL;

  if(!autonomous_mode)
    if(p3_y < 127 - 12 || p3_y > 127 + 12 || p3_sw_aux1 || p3_sw_aux2)
      armstate = ARM_MANUAL;
    else if(p4_sw_aux1)
      armstate = ARM_PICK_UP;
    else if(p4_sw_top)
      armstate = ARM_HURDLE;
    else if(p4_sw_aux2)
      ;//armstate = ARM_POKE;

  //printf("state %d ", armstate);
  printf("armpot %d\n", armpot);

  if(armstate == ARM_MANUAL){
    ARM = ARM_JOY;

    //do big tilt piston
    if(p3_sw_aux1)
      TILT_F = 0;
    if(p3_sw_aux2)
      TILT_F = 1;
  }
  else{
    //do arm position
    switch(armstate){
    case ARM_PICK_UP:
      gotopos = ARM_PICKUP_POS;
      //arm should be lowered to pick up
      TILT_F = 0;
      break;
    case ARM_HURDLE:
      gotopos = ARM_HURDLE_POS;
      //arm should be raised to hurdle
      TILT_F = 1;
      break;
    case ARM_POKE:
      gotopos = ARM_POKE_POS_LOW;
      //arm should be raised to prepare to poke
      TILT_F = 1;
      break;
    }

    if(INRANGE(armpot, gotopos, 4)) //I is annoying
      armpid.error_sum = INITIAL_VALUE;

    c = controller(&armpid, armpot, gotopos);

    ARM = 127 + c;
    printf("armgotopos %d\n", gotopos);
  }
  TILT_R = !TILT_F;

  /*
  //1843: 2048*9/10
  //205: 2048*1/10
  if(armpot > 1843 && ARM > 127) //keep it from going too low
    ARM = 127;
  if(armpot < 205 && ARM < 127) //too high
    ARM = 127;
  */
  printf("armout %d\n", ARM);
}

void do_puncher(){
  /*
  static char solstate = 0;
  static int t;
  //0: off
  //1: puncher on, vacuum off (20 loops)
  //2: puncher off, vacuum on (60 loops)
  switch(solstate){
  case 0:
    VAC = 0;
    PUNCH_F = 0;
    if(PUNCH_BUTTON){
      t = 0;
      solstate = 1;
    }
    break;
  case 1:
    VAC = 0;
    PUNCH_F = 1; 
   t++;
    if(t >= 20){
      t = 0;
      solstate = 2;
    }
    break;
  case 2:
    VAC = 1;
    PUNCH_F = 0;
    t++;
    if(t >= 180)
      solstate = 0;
    break;
  }

  if(solstate == 0){ //run vacuum when grabber open, but only while idling
    VAC = GRAB_BUTTON;
  }
  */
  static char state = 3;
  static unsigned int t = 0;

  switch(state){
  case 0: //put puncher out, but keep it in with the pin
    FIREPIN_F = 1;
    PUNCH_F = 1;
    if(PUNCH_BUTTON){
      t = FIREPIN_F = 0;
      state = 1;
    }
    break;
  case 1: //retract pin, let puncher out
    FIREPIN_F = 0;
    t++;
    if(t >= 19)
      state = 2;
    break;
  case 2: //retract puncher
    PUNCH_F = 0;
    if(PUNCH_SWITCH){
      state = 3;
      t = 0;
    }
    break;
  case 3: //put pin out
    FIREPIN_F = 1;
    t++;
    if(t >= 19)
      state = 0;
    break;
  }
  FIREPIN_R = !FIREPIN_F;
  
  //PUNCH_F = PUNCH_BUTTON;
}
