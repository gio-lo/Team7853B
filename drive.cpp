#include "drive.h"
#include "main.h"
#include "utils.h"

// Applies easing to joystick values for smoother motion
int ease(int jsVal) {
  int v = (int)(((jsVal < 0) ? -1 : 1) * EASE_COEFFICIENT * jsVal * jsVal);
  return v;
}

// Drives a single motor forward or backward at a velocity percentage
void driveMotor(motor mot, int vel) {
  bool isReversed = (vel < 0);
  double pctvel = (abs(vel) / 127.0) * 100.0;
  mot.setReversed(isReversed);
  mot.spin(fwd, pctvel, pct);
}

// Stops all drive motors
void driveStop() {
  frontLeftMotor.stop();
  frontRightMotor.stop();
  backLeftMotor.stop();
  backRightMotor.stop();
}

// Drives robot forward for a set duration
void driveForward(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, velocitynum);
  driveMotor(backLeftMotor, velocitynum);
  driveMotor(frontRightMotor, -velocitynum);
  driveMotor(backRightMotor, -velocitynum);
  task::sleep(ms);
  driveStop();
}

// Drives robot backward for a set duration
void driveBackward(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, -velocitynum);
  driveMotor(backLeftMotor, -velocitynum);
  driveMotor(frontRightMotor, velocitynum);
  driveMotor(backRightMotor, velocitynum);
  task::sleep(ms);
  driveStop();
}

// Strafes left using mecanum wheels
void driveLeft(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, -velocitynum);
  driveMotor(backLeftMotor, velocitynum);
  driveMotor(frontRightMotor, -velocitynum);
  driveMotor(backRightMotor, velocitynum);
  task::sleep(ms);
  driveStop();
}

// Strafes right using mecanum wheels
void driveRight(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, velocitynum);
  driveMotor(backLeftMotor, -velocitynum);
  driveMotor(frontRightMotor, velocitynum);
  driveMotor(backRightMotor, -velocitynum);
  task::sleep(ms);
  driveStop();
}

// Drives forward for a set distance in inches
void driveDistanceForward(double inches, int velocitynum) {
  double goalrevs = frontLeftMotor.rotation(rev) + (inches / (2.0 * 2.0 * PI));
  while (frontLeftMotor.rotation(rev) < goalrevs) {
    driveForward(velocitynum, 10);
  }
}

// Drives backward for a set distance in inches
void driveDistanceBackward(double inches, int velocitynum) {
  double goalrevs = frontLeftMotor.rotation(rev) + (inches / (2.0 * 2.0 * PI));
  while (frontLeftMotor.rotation(rev) < goalrevs) {
    driveBackward(velocitynum, 10);
  }
}

// Strafes left for a set distance
void driveDistanceLeft(double inches, int velocitynum) {
  double goalrevs = frontLeftMotor.rotation(rev) + (inches / (2.0 * 2.0 * PI));
  while (frontLeftMotor.rotation(rev) < goalrevs) {
    driveLeft(velocitynum, 10);
  }
}

// Strafes right for a set distance
void driveDistanceRight(double inches, int velocitynum) {
  double goalrevs = frontLeftMotor.rotation(rev) + (inches / (2.0 * 2.0 * PI));
  while (frontLeftMotor.rotation(rev) < goalrevs) {
    driveRight(velocitynum, 10);
  }
}

// Turns robot left for a duration
void turnLeft(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, -velocitynum);
  driveMotor(backLeftMotor, -velocitynum);
  driveMotor(frontRightMotor, -velocitynum);
  driveMotor(backRightMotor, -velocitynum);
  task::sleep(ms);
  driveStop();
}

// Turns robot right for a duration
void turnRight(int velocitynum, int ms) {
  velocitynum = abs(velocitynum);
  driveMotor(frontLeftMotor, velocitynum);
  driveMotor(backLeftMotor, velocitynum);
  driveMotor(frontRightMotor, velocitynum);
  driveMotor(backRightMotor, velocitynum);
  task::sleep(ms);
  driveStop();
}

// Turns left to a target gyro degree
void turnLeft(int velocitynum, double degree) {
  newSpinny.resetHeading();
  currentGyro = newSpinny.yaw(deg);
  double target = degree + 0.5;
  while (currentGyro < target) {
    turnLeft(velocitynum, 20);
    currentGyro = newSpinny.yaw(deg);
  }
}

// Turns right to a target gyro degree
void turnRight(int velocitynum, double degree) {
  newSpinny.resetHeading();
  currentGyro = newSpinny.yaw(deg);
  double target = degree - 0.5;
  while (currentGyro > target) {
    turnRight(velocitynum, 20);
    currentGyro = newSpinny.yaw(deg);
  }
}

// PID-controlled turn to specific degree
void turnDegree(double degree) {
  bool goLeft = false, goRight = false;
  double error = 2.0;
  currentGyro = newSpinny.heading(deg);

  if (degree < 0) degree += 360;
  double upper = fmod(degree + error, 360);
  double lower = fmod(degree - error + 360, 360);

  if (currentGyro < degree - 0.5) {
    goRight = abs(currentGyro - degree) < 180;
  } else {
    goLeft = abs(currentGyro - degree) < 180;
  }

  while ((currentGyro < lower || currentGyro > upper)) {
    double speed = pid(degree, currentGyro, 2.5, 0, 0);
    if (goLeft)
      turnLeft(speed, 10);
    else
      turnRight(speed, 10);
    currentGyro = newSpinny.heading(deg);
  }
  driveStop();
}

// PID-controlled forward/backward distance driving with straight correction
void driveDistance(double inches) {
  double revs = std::abs(inches) / (5.5 * PI);
  double avgEncoder = (-1 * encoderLeft.rotation(rev));
  double goalrevs = avgEncoder + revs;
  double setGyro = newSpinny.heading(deg);

  if (inches > 0) {
    while (avgEncoder < goalrevs) {
      driveForward(pid(goalrevs, avgEncoder, 250, 0, 0), 20);
      avgEncoder = (-1 * encoderLeft.rotation(rev));
    }
  } else {
    while (avgEncoder < goalrevs) {
      double straightGyro = newSpinny.heading(deg);
      if (abs(setGyro - straightGyro) > 2) {
        turnDegree(setGyro);
      }
      driveBackward(pid(goalrevs, avgEncoder, 5, 0, 0), 20);
      avgEncoder = (-1 * encoderLeft.rotation(rev));
    }
  }
  turnDegree(setGyro);
}
