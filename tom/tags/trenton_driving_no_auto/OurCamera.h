#ifndef __OURCAMERA_H
#define __OURCAMERA_H

#define MIN_CONF 40
#define MAX_CONF 100
#define MIN_BLOB_SIZE 30

/*#define _DEBUG

#ifdef _DEBUG
#define DEBUG(x) printf x
#else
#define DEBUG(x)
#endif*/

int check_multiple_lights();
void split_window(int side);

#endif
