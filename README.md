# VEX V5 Robotics Control System

This project contains a modular control system for a VEX V5 robot, featuring mecanum drive, arm and claw control, and computer vision-based object detection and alignment.

---

##  Features

- **Mecanum Drive Control** with joystick-based inputs
- **Arm & Claw Movement** using preset revolutions or timed control
- **Inertial Sensor Turns** for accurate rotation
- **Vision Sensor Alignment** for object tracking and cube scoring
- **PID Control** for smooth, responsive driving and alignment
- **Modular Code Structure** (easy to extend, maintain, and understand)

---

##  File Descriptions
src files
├── main.cpp # Entry point for autonomous and driver modes 
├── drive.cpp # All drive-related logic (forward, turn, strafe, etc.) 
├── arms.cpp # Arm and claw movement logic 
├── vision_helpers.cpp # Vision sensor tracking and alignment 
├── utils.cpp # PID controller logic

include files
├── main.h # Global declarations and hardware constants 
├── drive.h # Drive function declarations 
├── arms.h # Arm/claw function declarations 
├── vision_helpers.h # Vision utility declarations 
├── utils.h # PID function declaration 
├── vex.h # VEX-provided include 
├── vision.h # Vision sensor signature config

---

## Credits

Developed by Giordan Lozott, Erik Liang, Nikola Tunguz  
Originally built as a high school robotics project  
Refactored and modularized for presentation in 2025.
