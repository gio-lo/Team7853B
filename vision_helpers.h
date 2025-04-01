#ifndef VISION_HELPERS_H
#define VISION_HELPERS_H

#include "vex.h"
#include <string>

// Vision detection and drawing
void detectColor(std::string cubeColor);
void draw();

// Vision-based alignment using gyro and object location
void horizontalAlign(std::string cubeColor);
void forwardAlign(std::string cubeColor);

// Object motion tracking and following
void updateArray(int cycle);
void followBall();

#endif
