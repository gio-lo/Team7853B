#ifndef DRIVE_H
#define DRIVE_H


#include "vex.h"
#include <iostream>


//forward declarations for functions
//basic drive Functions
 int ease(int jsVal);
 void driveMotor(motor mot, int vel);
 void driveStop();


//driving in specific directions
 void driveBackward(int velocitynum,int ms );
 void driveForward(int velocitynum,int ms );
 void driveLeft(int velocitynum, int ms);
 void driveRight(int velocitynum, int ms );


//driving specific amount of inches forward
 void driveDistanceRight(double inches, int velocitynum);
 void driveDistanceLeft(double inches, int velocitynum);
 void driveDistanceForward(double inches, int velocitynum);
 void driveDistanceBackward(double inches, int velocitynum);
  void driveDistance(double inches);




//basic arm motions on time and revolutions
 void lowerArms(int ms);
 void liftArms(int ms);
 void armsRevolutions(double revs, int velocitynum);


//function declaration
 void closeClaw(int ms);
 void openClaw(int ms);


//functions for turning left / right by time
 void turnLeft(int velocitynum, int ms);
 void turnRight(int velocitynum , int ms);


//functions for turning a specific amount of degrees
 void turnDegree(double degree);
 void turnRight(int velocitynum,double degree);
 void turnLeft(int velocitynum,double degree);




//vision sensing related functions and alignment
 void draw();
 void detectColor( std::string cubeColor);
 void horizontalAlign( std::string cubeColor );
 void forwardAlign( std ::string cubeColor);


//following an object related functions and constants
 void updateArray(int cycle);
 void followBall();


//a test to display gyro values
void gyroTest();






 void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle);


#endif
