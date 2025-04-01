#include "vision_helpers.h"
#include "main.h"
#include "drive.h"
#include "utils.h"

// Draws a red rectangle on the Brain screen over the detected object
void draw() {
  Brain.Screen.clearScreen();
  Brain.Screen.setOrigin(1, 1);
  Brain.Screen.drawRectangle(0, 0, 316, 212);
  Brain.Screen.drawRectangle(
    Vision1.largestObject.originX,
    Vision1.largestObject.originY,
    Vision1.largestObject.width,
    Vision1.largestObject.height,
    color::red
  );
}

// Captures and processes the specified color signature
void detectColor(std::string cubeColor) {
  if (cubeColor == "ORA") Vision1.takeSnapshot(ORA);
  else if (cubeColor == "GRE") Vision1.takeSnapshot(GRE);
  else if (cubeColor == "PUR") Vision1.takeSnapshot(PUR);

  objectAngle = ((atan((158 - Vision1.largestObject.centerX) / 205.910)) * (180 / PI));
  objectAngle2 = ((atan((106.5 - Vision1.largestObject.height) / 6.6056)) * (180 / PI));
  distanceobj = 6.7 * tan(objectAngle2);
  currentGyro = newSpinny.yaw(rotationUnits::deg);
  // draw(); // Optional debug visualization
}

// Aligns robot horizontally to center a cube
void horizontalAlign(std::string cubeColor) {
  detectColor(cubeColor);

  if (Vision1.largestObject.exists) {
    while (isNotCentered) {
      detectColor(cubeColor);
      if (currentGyro > objectAngle + currentGyro + offset1) {
        driveRight(pid(objectAngle + currentGyro, currentGyro, horKP, horKI, horKD), 20);
      } else if (currentGyro < objectAngle + currentGyro - offset1) {
        driveLeft(pid(objectAngle + currentGyro, currentGyro, horKP, horKI, horKD), 20);
      } else {
        break;
      }

      isNotCentered = !(currentGyro > objectAngle + currentGyro + offset1) ||
                      !(currentGyro < objectAngle + currentGyro - offset1);
    }
    driveStop();
  }
}

// Aligns robot forward/backward to position near a cube
void forwardAlign(std::string cubeColor) {
  detectColor(cubeColor);

  if (Vision1.largestObject.exists) {
    while (isNotClose) {
      detectColor(cubeColor);

      if (Vision1.largestObject.width > goal + offsetX) {
        driveBackward(pid(goal, Vision1.largestObject.width, vertKP, vertKI, vertKD), 20);
      } else if (Vision1.largestObject.width < goal - offsetX) {
        driveForward(pid(goal, Vision1.largestObject.width, vertKP, vertKI, vertKD), 20);
      } else {
        break;
      }

      isNotClose = !(Vision1.largestObject.width > goal + offsetX) ||
                   !(Vision1.largestObject.width < goal - offsetX);
    }
    driveStop();
  }
}

// Updates rolling object location history (X, Y, Area)
void updateArray(int cycle) {
  Vision1.takeSnapshot(GRE);
  if (Vision1.largestObject.exists) {
    curindexArr = indexArr % 100;
    previousLocations[curindexArr][0] = Vision1.largestObject.centerX;
    previousLocations[curindexArr][1] = Vision1.largestObject.centerY;
    previousLocations[curindexArr][4] = Vision1.largestObject.width * Vision1.largestObject.height;

    if (indexArr >= cycle) {
      previousLocations[curindexArr][2] = previousLocations[curindexArr][0] - previousLocations[(curindexArr - cycle) % 100][0];
      previousLocations[curindexArr][3] = previousLocations[curindexArr][1] - previousLocations[(curindexArr - cycle) % 100][1];
      previousLocations[curindexArr][5] = previousLocations[curindexArr][4] - previousLocations[(curindexArr - cycle) % 100][4];
    }
    indexArr++;
  }
}

// Follows the object using simple motion prediction + feedback loop
void followBall() {
  double areaGoal = 1000.0;
  double xGoal = 158.0;

  int x = previousLocations[curindexArr][0];
  int area = previousLocations[curindexArr][4];
  int changeX = previousLocations[curindexArr][2];
  int changeA = previousLocations[curindexArr][5];

  double errorArea = abs(area - areaGoal);
  double errorX = abs(x - xGoal);
  double priorityTurn = errorX / xGoal;
  double priorityForward = errorArea / areaGoal;
  if (priorityForward > 1) priorityForward -= 1;
  bool goForward = priorityTurn < priorityForward;

  double areaAverageTotal = 0, xAverageTotal = 0;
  for (int i = 4; i >= 0; i--) {
    areaAverageTotal += previousLocations[curindexArr - i][5];
    xAverageTotal += previousLocations[curindexArr - i][2];
  }
  double areaAverageChange = areaAverageTotal / 5;
  double xAverageChange = xAverageTotal / 5;

  bool goFarForward = (xAverageChange < 5 && xAverageChange > -5);

  if (Vision1.largestObject.exists && area > 200) {
    if (x > 168 && x < 200) {
      driveRight(pid(158, x, 0.5, 0, 0), 100);
    } else if (x < 148 && x > 116) {
      driveLeft(pid(158, x, 0.5, 0, 0), 100);
    } else if (x >= 200) {
      turnRight(pid(158, x, 0.5, 0, 0), 100);
    } else if (x <= 116) {
      turnLeft(pid(158, x, 0.5, 0, 0), 100);
    } else {
      task::sleep(100);
    }
  } else {
    if (xAverageChange < 2) {
      turnLeft(100, 100);
    } else if (xAverageChange > -2) {
      turnRight(100, 100);
    } else {
      task::sleep(100);
    }
  }
}
