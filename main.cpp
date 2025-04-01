#include "main.h"
#include "drive.h"
#include "arms.h"
#include "vision_helpers.h"
#include "utils.h"
#include "vision.h"

// Vision signature setup
vex::vision::signature PUR = vex::vision::signature(1, -341, 1323, 491, 7235, 10659, 8947, 1.2, 0);
vex::vision::signature ORA = vex::vision::signature(2, 5795, 7759, 6777, -2655, -1991, -2323, 2.9, 0);
vex::vision::signature GRE = vex::vision::signature(3, -6763, -4817, -5790, -5635, -3927, -4780, 2.6, 0);
vex::vision::signature SIG_4 = vex::vision::signature(4, 0, 0, 0, 0, 0, 0, 2, 0);
vex::vision::signature SIG_5 = vex::vision::signature(5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 = vex::vision::signature(6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 = vex::vision::signature(7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision Vision1 = vex::vision(vex::PORT19, 64, PUR, ORA, GRE, SIG_4, SIG_5, SIG_6, SIG_7);

// Global sensor and motor definitions
vex::competition Competition;
vex::brain Brain;
vex::controller Controller1;
vex::inertial newSpinny = vex::inertial(PORT17);
vex::motor frontLeftMotor(PORT20, ratio18_1, false);
vex::motor backLeftMotor(PORT10, ratio18_1, false);
vex::motor frontRightMotor(PORT11, ratio18_1, false);
vex::motor backRightMotor(PORT1, ratio18_1, false);
vex::motor rightArm(PORT7, ratio36_1, false);
vex::motor leftArm(PORT2, ratio36_1, false);
vex::motor grabby(PORT3, ratio18_1, false);
vex::encoder encoderLeft(Brain.ThreeWirePort.A);
vex::encoder encoderRight(Brain.ThreeWirePort.E);

// Constants
const double EASE_COEFFICIENT = 0.015;
const double SLOW_WHEELS_COF = 0.5;
const double SLOW_LIFT_COF = 0.4;
const double PI = 3.14159265359;

// Arm lift values
double revolutionTracker = 0.0;
double lowTowerLift = 1;
double midTowerLift = 1.187;
double highTowerLift = 2.1;
double oneBlock = 0.33;
double moreBlocks = 0.23;
double blockError = 0.07;

// Vision constants and tracking
bool setGyro = true;
int centerFOV = 158;
int offsetX = 15;
int offset1 = 1;
int goal = 140;
double lastError = 0;
double errorT = 0;
double vertKP = 0.38, vertKI = 0.0, vertKD = 0.0;
double horKP = 3.0, horKI = 0.0, horKD = 0.0;

int previousLocations[100][6];
int indexArr = 0;
int curindexArr;
double objectAngle;
double objectAngle2;
double distanceobj;
double currentGyro;
bool isNotCentered = true;
bool isNotClose = true;

// Pre-autonomous setup
void pre_auton(void) {
  newSpinny.startCalibration();
}

// Autonomous function (calls a selected routine)
void autonomous(void) {
  autoCases(2); // Example: running case 2 (Red/Big Tower + Stack)
}

// User control loop
void usercontrol(void) {
  int js1X, js1Y, js2X, js2Y;
  int ease1X, ease1Y, ease2X, ease2Y;
  int atc = 0, btc = 0;
  bool slowWheels = false;
  bool slowLift = false;

  while (true) {
    js1X = Controller1.Axis4.value();
    js1Y = Controller1.Axis3.value();
    js2X = Controller1.Axis1.value();
    js2Y = Controller1.Axis2.value();

    // Toggle arm speed
    if (Controller1.ButtonA.pressing() && atc <= 0) atc = 2;
    else if (atc > 0 && !Controller1.ButtonA.pressing()) {
      atc--;
      if (atc <= 0) slowLift = !slowLift;
    }

    // Toggle wheel speed
    if (Controller1.ButtonB.pressing() && btc <= 0) btc = 2;
    else if (btc > 0 && !Controller1.ButtonB.pressing()) {
      btc--;
      if (btc <= 0) slowWheels = !slowWheels;
    }

    // Apply easing and optional slow mode
    ease1X = ease(js1X * (slowWheels ? SLOW_WHEELS_COF : 1.0));
    ease1Y = ease(js1Y * (slowWheels ? SLOW_WHEELS_COF : 1.0));
    ease2X = ease(js2X * (slowWheels ? SLOW_WHEELS_COF : 1.0));
    ease2Y = ease(js2Y * (slowWheels ? SLOW_WHEELS_COF : 1.0));

    // Drive with mecanum logic
    driveMotor(frontLeftMotor, ease1Y + ease2X + ease1X);
    driveMotor(frontRightMotor, -ease1Y + ease2X + ease1X);
    driveMotor(backLeftMotor, ease1Y + ease2X - ease1X);
    driveMotor(backRightMotor, -ease1Y + ease2X - ease1X);

    // Arm control
    if (Controller1.ButtonL1.pressing()) {
      driveMotor(leftArm, -100 * (slowLift ? SLOW_LIFT_COF : 1.0));
      driveMotor(rightArm, 100 * (slowLift ? SLOW_LIFT_COF : 1.0));
    } else if (Controller1.ButtonL2.pressing()) {
      driveMotor(leftArm, 100 * (slowLift ? SLOW_LIFT_COF : 1.0));
      driveMotor(rightArm, -100 * (slowLift ? SLOW_LIFT_COF : 1.0));
    } else {
      leftArm.stop(brakeType::hold);
      rightArm.stop(brakeType::hold);
    }

    // Claw control
    if (Controller1.ButtonR2.pressing()) {
      driveMotor(grabby, -100);
    } else if (Controller1.ButtonR1.pressing()) {
      driveMotor(grabby, 100);
    } else {
      grabby.stop(brakeType::hold);
    }

    // Vision tracking: forward straight
    if (Controller1.ButtonUp.pressing()) {
      if (setGyro) {
        currentGyro = newSpinny.yaw(deg);
        setGyro = false;
      }
      if (newSpinny.yaw(deg) < currentGyro - 2) {
        turnRight(20, 50);
      } else if (newSpinny.yaw(deg) > currentGyro + 2) {
        turnLeft(20, 50);
      } else {
        driveForward(20, 100);
      }
    }

    // Vision object following
    if (Controller1.ButtonDown.pressing()) {
      updateArray(5);
      followBall();
    }

    task::sleep(20);
  }
}

// Main function — competition setup
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    task::sleep(100); // Prevent CPU overuse
  }
}
