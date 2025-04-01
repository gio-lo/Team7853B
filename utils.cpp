#include "utils.h"
#include "main.h"  // for errorT and lastError

// PID controller
// Calculates control signal based on proportional, integral, and derivative components
double pid(double goal, double currentValue, double kp, double ki, double kd) {
  double error = goal - currentValue;

  // Accumulate error only if it's non-zero
  if (error != 0) {
    errorT += error;
  } else {
    errorT = 0;
  }

  // Cap accumulated error to prevent runaway integral growth (anti-windup)
  if (errorT > 315) {
    errorT = 315;
  }

  // Derivative term (change in error)
  double derivative = (error - lastError);

  // Save for next frame
  lastError = error;

  // Compute PID output
  double proportional = error * kp;
  double integral = errorT * ki;
  derivative *= kd;

  return proportional + integral + derivative;
}
