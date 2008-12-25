#ifndef BUILTINS_H_
#define BUILTINS_H_

#include "API.h"

void SetMotor(unsigned char port, int speed);
void SetInvertedMotor(unsigned char port);
void Motor(unsigned char pwmPort, int speed);
void Motors(int leftSpeed, int rightSpeed);
void Drive(int speed, int direction);
void TwoWheelDrive(unsigned char _leftMotor, unsigned char _rightMotor);
void FourWheelDrive(unsigned char _leftMotor, unsigned char _frontLeftMotor,
					unsigned char _rightMotor, unsigned char _frontRightMotor);

#endif /*BUILTINS_H_*/
