#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "eeprom.h"
#include "record.h"
#include "common_def.h"

int t1, t2;

void record_reset(){
  t1 = t2 = 0;
}

void record(){
  t1++;

  Relay2_red = Relay2_green = 0;
  if(t1 < 1024)
    Relay2_red = 1;
  else
    Relay2_green = 1;

  if(t1 < 1024 && (t1&1)){ //record every other frame
    EEPROM_Write(t1 - 1, L_DRIVE1);
    EEPROM_Write(t1, R_DRIVE1);
    printf("writing %d %d %d %d\n", t1-1, L_DRIVE1, t1, R_DRIVE1);
  }
}

void replay(){
  static unsigned char x1, x2, y1, y2;
  t2++;

  if(t2 < 1024)
    if(t2 == 1){
      x2 = EEPROM_Read(0);
      y2 = EEPROM_Read(1);
    }

    if(t2&1){ //odd: play recorded value
      L_DRIVE1 = L_DRIVE2 = x2;//EEPROM_Read(t2 - 1);
      R_DRIVE1 = R_DRIVE2 = y2;//EEPROM_Read(t2);
    }
    else{ //even: read new value, play average
      x1 = x2;
      y1 = y2;

      x2 = EEPROM_Read(t2);
      y2 = EEPROM_Read(t2 + 1);

      printf("read %d %d %d %d\n", t2, x2, t2+1, y2);

      /*
      L_DRIVE1 = L_DRIVE2 = (unsigned char)(((int)x1 + (int)x2)/2);
      R_DRIVE1 = R_DRIVE2 = (unsigned char)(((int)y1 + (int)y2)/2);
      */
      L_DRIVE1 = L_DRIVE2 = x2;//EEPROM_Read(t2 - 1);
      R_DRIVE1 = R_DRIVE2 = y2;//EEPROM_Read(t2);
      
    }
}
