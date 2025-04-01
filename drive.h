#ifndef DRIVE_H
#define DRIVE_H

#include "vex.h"

// Eases joystick input for smoother control
int ease(int jsVal);

// Drive motor control
void driveMotor(motor mot, int vel);
void driveStop();

// Time-based drive motion
void driveForward(int velocitynum, int ms);
void driveBackward(int velocitynum, int ms);
void driveLeft(int velocitynum, int ms);
void driveRight(int velocitynum, int ms);

// Distance-based drive motion
void driveDistanceForward(double inches, int velocitynum);
void driveDistanceBackward(double inches, int velocitynum);
void driveDistanceLeft(double inches, int velocitynum);
void driveDistanceRight(double inches, int velocitynum);
void driveDistance(double inches);

// Turn using time or gyro degrees
void turnLeft(int velocitynum, int ms);
void turnRight(int velocitynum, int ms);
void turnLeft(int velocitynum, double degree);
void turnRight(int velocitynum, double degree);
void turnDegree(double degree);

// Mecanum drive
void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle);

#endif
