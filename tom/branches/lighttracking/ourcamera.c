#include "camera.h"
#include "tracking.h"
#include "ourcamera.h"
#include <stdio.h>

extern unsigned int camera_t_packets;
extern unsigned int camera_acks;

int x1,y1,x2,y2;

int check_multiple_lights()
{
  
  if(T_Packet_Data.confidence > MIN_CONF && T_Packet_Data.confidence < MAX_CONF)
    {
      printf("Multiple lights\r\n");
      return 1;
    }
  else 
    return 0;

}
  
void split_window(int side)
{ 
  static char waiting_for_ack=0;
  static int old_camera_t_packets=0;
  static unsigned int old_acks=0;
  static int state=STATE_INACTIVE;
  static int timeout=0;
  int centroid;

  printf("Acks: %d\r\n",camera_acks);

  if(waiting_for_ack)
    {
      //printf("Current: %d Old: %d\r\n",camera_acks,old_acks);
      if(camera_acks != old_acks)
	{
	  waiting_for_ack = 0;
	  printf("Got ack: state %d\r\n",state);
	}
      else
	{
	  return;
	}
    }
  old_acks = camera_acks;
  switch(state)
    {
    case STATE_INACTIVE:
      if(old_camera_t_packets != camera_t_packets)
	{
	  if(check_multiple_lights())
	    state = STATE_POLL_ON;
	  old_camera_t_packets = camera_t_packets;
	}
      break;
    case STATE_POLL_ON:
      Poll_Mode(1);
      waiting_for_ack = 1;
      state = STATE_SET_WINDOW;
      break;
    case STATE_SET_WINDOW:  
      centroid = T_Packet_Data.mx;
      Camera_Idle();
      if(side==1)
	{
	  Virtual_Window(1,1,centroid,IMAGE_HEIGHT);
	  waiting_for_ack=1;
	  printf("Using virtual window\r\n");
	    }
      else
	{
	  Camera_Idle();
	  Virtual_Window(centroid,1,IMAGE_WIDTH,IMAGE_HEIGHT);
	  waiting_for_ack=1;
	  printf("Using virtual window\r\n");
	}
      waiting_for_ack = 1;
      break;
    case STATE_POLL_OFF:
      Poll_Mode(0);
      waiting_for_ack = 1;
      state = STATE_INACTIVE;
      break;
    }
}
//  }
      
    // Track_Color(R_MIN_DEFAULT, R_MAX_DEFAULT,G_MIN_DEFAULT, G_MAX_DEFAULT,B_MIN_DEFAULT, B_MAX_DEFAULT);
