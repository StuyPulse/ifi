#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "controller.h"
#include "gyrodrive.h"
#include "user_routines.h"
#include "common_def.h"
#include "gyro.h"
#include "velocity.h"



/*
  0: slow down some distance before reaching the turn
  1: set left wheel to a negative value while turning
  2: use velocity functions to actually control speed
*/
#define TURNMODE 0

/*
  0: go straight and stop
  else: do laps
*/
#define MODE 1

struct pidstate pid_str_b, pid_turn_b; //conflict with pid_str and pid_turn of ir.c!

#if TURNMODE==2
struct pidstate vel_pid;
#endif

extern int left_ticks, right_ticks;
extern char ir;

void init_gyro_pids(){
  /*
  init_controller(&pid_str_b,  1, 25, 0, 1000, 1, 30, 50, -50);
  init_controller(&pid_turn_b, 1, 25, 0, 1,    2, 5, 50, -50);
  */

  init_controller(&pid_str_b,  1, 8,    0, 250,   2, 1,   50, -50);
  init_controller(&pid_turn_b, 1, 10,   0, 1,     1, 20,  60, -60);
  //init_controller(&pid_turn_b, 1, 40,   0, 1,     2, 5,   50, -50);
#if TURNMODE==2
  init_controller(&vel_pid, 1, 500, 1, 10000, 
#endif
}

#if MODE==0
//just go straight and stop
void Gyro_Drive(void){
#define STRAIGHT_DIST1 420 //540
#define STRAIGHT_TICKS1 (STRAIGHT_DIST1 / WHEEL_CIRC * NUM_DRIVE_SPOKE)
  //#define STRAIGHT_DIST2 36//96
  //#define STRAIGHT_TICKS2 (STRAIGHT_DIST2 / WHEEL_CIRC * NUM_DRIVE_SPOKE)

  int ang = Get_Gyro_Angle();
  char c = 0;
  static int ang0 = 0;
  static int n, state = -1;
  int i;
  static char go = 1;

  printf("ir %d go %d\n", ir, go);

  //stop moving on any button
  if(ir)
    go = 0;
  if(!go){
    L_DRIVE1 = L_DRIVE2 = R_DRIVE1 = R_DRIVE2 = 127;
    return;
  }

  TILT_F = 1;
  TILT_R = 0;

  /*
    state:
    0: starting a long straightaway
    1: middle of a straightaway
    2: stop
  */
  
  //printf("ang0 %d ang %d\n", ang0, ang);
  //printf("err %d\n", pid.error_sum);

  if(state == -1){
    n = (int)STRAIGHT_TICKS1;
    state = 0;
  }
  printf("ir %d\n", ir);
  printf("gyrodrive state %d\n", state);
  printf("ang %d ang0 %d\n", ang, ang0);
  printf("left %d right %d\n", left_ticks, right_ticks);
  
  switch(state){
  case 0: //start straight
    //ang0 = ang;
    left_ticks = right_ticks = 0;
    pid_str_b.error_sum = INITIAL_VALUE; //reset
    state = 1;
    //n = (int)STRAIGHT_TICKS1 + (int)STRAIGHT_TICKS2 - n;
  case 1:
    c = (char)controller(&pid_str_b, ang, ang0);
    R_DRIVE1 = R_DRIVE2 = 127 - 70 - c;
    L_DRIVE1 = L_DRIVE2 = 127 + 70 - c;

    if((left_ticks + right_ticks) >= 2*n) //distance traveled
      state = 2;

    printf("c %d error %d\n", c, pid_str_b.last_error);

    break;

  case 2:
    R_DRIVE1 = R_DRIVE2 = L_DRIVE1 = L_DRIVE2 = 127;
    break;
  }
}

#else
//do laps
void Gyro_Drive(void){
#define STRAIGHT_DIST1 504 //42 feet, the total distance to go straight
#define STRAIGHT_TICKS1 (STRAIGHT_DIST1 / WHEEL_CIRC * NUM_DRIVE_SPOKE)
#define STRAIGHT_DIST2 96 // 6 feet, the distance to go after turning
#define STRAIGHT_TICKS2 (STRAIGHT_DIST2 / WHEEL_CIRC * NUM_DRIVE_SPOKE)
#define STRAIGHT_DIST3 432 //36 feet, the distance to go before slowing down
#define STRAIGHT_TICKS3 (STRAIGHT_DIST2 / WHEEL_CIRC * NUM_DRIVE_SPOKE)

  int ang = Get_Gyro_Angle();
  char c = 0;
  static int ang0 = 0;
  static int n, state = -1;
  int i;
  static char go = 1;
  int lv, rv;
  

  printf("ir %d go %d\n", ir, go);

  //stop moving on any button
  if(ir)
    go = 0;
  if(!go){
    L_DRIVE1 = L_DRIVE2 = R_DRIVE1 = R_DRIVE2 = 127;
    return;
  }

  TILT_F = 1;
  TILT_R = 0;

  /*
    state:
    0: starting a long straightaway
    1: middle of a straightaway
    2: starting a turn
    3: middle of a turn
  */
  
  //printf("ang0 %d ang %d\n", ang0, ang);
  //printf("err %d\n", pid.error_sum);

  if(state == -1){
    n = (int)STRAIGHT_TICKS2;
    state = 0;
  }

  printf("gyrodrive state %d\n", state);
  printf("ang %d ang0 %d\n", ang, ang0);
  printf("left %d right %d\n", left_ticks, right_ticks);
  
  switch(state){
  case 0: //start straight
    //ang0 = ang;
    left_ticks = right_ticks = 0;
    pid_str_b.error_sum = INITIAL_VALUE; //reset
    state = 1;
    n = (int)STRAIGHT_TICKS1 + (int)STRAIGHT_TICKS2 - n;
  case 1:
    c = (char)controller(&pid_str_b, ang, ang0);
    L_DRIVE1 = L_DRIVE2 = 127 + 60 - c;
    R_DRIVE1 = R_DRIVE2 = 127 - 60 - c;

#if TURNMODE==0
    if((left_ticks + right_ticks) >= 2*((int)STRAIGHT_TICKS3)){ //slow down
      L_DRIVE1 = L_DRIVE2 = L_DRIVE1 - 40;
      R_DRIVE1 = R_DRIVE2 = R_DRIVE1 + 40;
    }
#endif

    if((left_ticks + right_ticks) >= 2*n) //distance traveled
      state = 2;
    printf("c %d error %d\n", c, pid_str_b.last_error);
    break;

  case 2: //start turn
    state = 3;
  case 3:
    if(ang >= ang0 + 3142 * 9/10){ //finished turn?
      state = 0;
      ang0 += 3142;
      break;
    }
#if TURNMODE==1
    //meh
    L_DRIVE1 = L_DRIVE2 = 116; //drive this one backwards a little; counteract momentum
    R_DRIVE1 = R_DRIVE2 = 0; //full forward
#elif TURNMODE==2
    lv = get_rpm_1k(LEFT_SIDE);
    rv = get_rpm_1k(RIGHT_SIDE);

    c = controller(&vel_pid, lv, );

    L_DRIVE1 = L_DRIVE2 = 

    c = controller(&vel_pid, lv, );

#else
    c = (char)controller(&pid_turn_b, ang, ang + 850);
    L_DRIVE1 = L_DRIVE2 = 127 + 60 - c;
    R_DRIVE1 = R_DRIVE2 = 127 - 60 - c;
#endif

    break;
  }
}
#endif
