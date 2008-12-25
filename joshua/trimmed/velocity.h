
#ifndef _VELOCITY_H
#define _VELOCITY_H

#define SAMPLES_AVERAGED_DRIVE 8

#define FLUCTUATION_DRIVE 15

//int left_velocity(void);
//int right_velocity(void);
long get_velocity(int);
long get_rpm_1k(int side);
#endif
