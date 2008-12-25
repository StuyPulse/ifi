#include "camera.h"
#include "tracking.h"
#include "OurCamera.h"

extern int camera_acks;

int check_multiple_lights()
{
  DEBUG(("Multiple lights\r\n"));
  return T_Packet_Data.confidence > MIN_CONF && T_Packet_Data.confidence < MAX_CONF;
}

void split_window(int side)
{
  static int old_camera_t_packets = 0;
  static int old_acks=0;
  int centroid;
  Polled_Mode();
  if(old_acks==0)
    old_acks = camera_acks;
  if(camera_acks == old_acks)
      return;
  else
    {
      printf("Got ack\r\n");
      old_acks = camera_acks;
    }
  
  if(camera_t_packets != old_camera_t_packets)
    {
      old_camera_t_packets = camera_t_packets;
      if(!check_multiple_lights())
	return;
      centroid = T_Packet_Data.mx;
      if(side==1)
	{
	  Virtual_Window(1,1,centroid,IMAGE_HEIGHT);
	  printf("Using virtual window\r\n");
	}
      else
	{
	  Virtual_Window(centroid,1,IMAGE_WIDTH,IMAGE_HEIGHT);
	  printf("Using virtual window\r\n");
	}
  
    }
  else
    Track_Color(R_MIN_DEFAULT, R_MAX_DEFAULT,G_MIN_DEFAULT, G_MAX_DEFAULT,B_MIN_DEFAULT, B_MAX_DEFAULT);
}
