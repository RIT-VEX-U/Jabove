#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::inertial imu;
// Analog sensors


// ================ OUTPUTS ================
// Motors
extern vex::motor_group intake_motors;
extern vex::motor_group left_motors;
extern vex::motor_group right_motors;

// Pneumatics

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;

// ================ UTILS ================

void robot_init();