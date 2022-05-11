#ifndef controll_H
#define controll_H

#define P_GAIN_R(a)  (a * 1 / (33.0) )
#define D_GAIN_R(a,b)  ((a-b) / 4.0)

#define P_GAIN_L(a)  (a * 1 / (33.0) )
#define D_GAIN_L(a,b)  ((a-b) / 4.0)

#define P_GAIN_F(a)  (a / 100.0)
#define D_GAIN_F(a,b)  ((a-b) / 4.0)

float pd_vel_control(int side, float last_out, float desired, float actual);
float ramp_coefficient(int curr_time, int max_time, int ramp_time);

#endif
