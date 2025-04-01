#ifndef ARMS_H
#define ARMS_H

#include "vex.h"

// Function declarations for controlling arms and claw

// Move arms to a specific number of motor revolutions
void armsRevolutions(double revs, int velocitynum);

// Time-based arm motion
void liftArms(int ms);
void lowerArms(int ms);

// Time-based claw control
void openClaw(int ms);
void closeClaw(int ms);

#endif