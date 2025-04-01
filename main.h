#ifndef MAIN_H
#define MAIN_H
#include "vision.h"
#include "vex.h"
#include <iostream>


//These are all of our physical component declarations, besides the vision sensor in its own file
extern vex::competition Competition;


extern vex::brain       Brain;
extern controller       Controller1;
//extern gyro             spinnyTool;
extern inertial         newSpinny;


extern motor            frontLeftMotor;
extern motor            backLeftMotor;
extern motor            frontRightMotor;
extern motor            backRightMotor;


extern motor            rightArm;
extern motor            leftArm;


extern motor            grabby;


extern encoder          encoderLeft;
extern encoder          encoderRight;




//constants for all of our functions
//these are our constants that we use to apply a slow moving mechanism
 extern const double EASE_COEFFICIENT;
 extern const double SLOW_WHEELS_COF;
 extern const double SLOW_LIFT_COF;


//constants for lifting arms revolutions
 extern double revolutionTracker;


 extern double lowTowerLift;
 extern double midTowerLift;
 extern double highTowerLift;
 extern double oneBlock;
 extern double moreBlocks;
 extern double blockError;


//constants for our Vision Sensor variables and application
 extern bool setGyro;
 extern int centerFOV;
 extern int offsetX;
 extern int offset1;
 extern int goal;
 extern double lastError;
 extern double errorT;
 extern const double PI;


//variables to apply the PID to alignment on a cube
 extern double vertKP;
 extern double vertKI;
 extern double vertKD;
 extern double horKP;
 extern double horKI;
 extern double horKD;


//variables to follow a cube
 extern int previousLocations[100][6];
 extern int indexArr;
 extern int curindexArr;


//variables to figure out per each object detecting on vision sensor
 extern double objectAngle;
 extern double objectAngle2;
 extern double distanceobj;
 extern double currentGyro;


//boolean checks to run functions on vision
 extern bool isNotCentered;
 extern bool isNotClose;






//these are forward declarations of some functions used in both cpp files
//declaration of our P.I.D.
 double pid(double goal1,double currentvalue, double kp, double ki, double kd);
//declaration of a function to score a tower
 void towerScore(std::string cubeColor , double towerHeight);
//our autonomous function to run all autos
 void autoCases(int caseNum);


#endif
