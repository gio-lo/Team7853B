#ifndef UTILS_H
#define UTILS_H

#include "vex.h"

// PID controller for distance and turning
// Returns velocity adjustment based on error between goal and current value
// Parameters: goal, current value, kp, ki, kd
// Returns: control output (velocity)
double pid(double goal, double currentValue, double kp, double ki, double kd);

#endif
