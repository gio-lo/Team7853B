#include "arms.h"
#include "main.h"

// Moves the arms to a specific position in motor revolutions
void armsRevolutions(double revs, int velocitynum) {
  // Stop motors before starting
  leftArm.stop(brakeType::hold);
  rightArm.stop(brakeType::hold);

  // Set motor directions
  leftArm.setReversed(true);
  rightArm.setReversed(false);

  // Move to specified revolutions
  leftArm.rotateTo(revs, rotationUnits::rev, velocitynum, velocityUnits::pct, false);
  rightArm.rotateTo(revs, rotationUnits::rev, velocitynum, velocityUnits::pct, true);

  // Stop excess movement
  leftArm.stop(brakeType::hold);
  rightArm.stop(brakeType::hold);
}

// Lifts the arms for a set amount of time, then holds position
void liftArms(int ms) {
  driveMotor(leftArm, -50);
  driveMotor(rightArm, 50);
  task::sleep(ms);
  leftArm.stop(brakeType::hold);
  rightArm.stop(brakeType::hold);
}

// Lowers the arms for a set amount of time, then holds position
void lowerArms(int ms) {
  driveMotor(leftArm, 50);
  driveMotor(rightArm, -50);
  task::sleep(ms);
  leftArm.stop(brakeType::hold);
  rightArm.stop(brakeType::hold);
}

// Opens the claw using negative motor power for a set duration
void openClaw(int ms) {
  driveMotor(grabby, -127);
  task::sleep(ms);
  grabby.stop(brakeType::hold);
}

// Closes the claw using positive motor power for a set duration
void closeClaw(int ms) {
  driveMotor(grabby, 127);
  task::sleep(ms);
  grabby.stop(brakeType::hold);
}
