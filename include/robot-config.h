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
extern PID drive_pid;

void intake(double volts);
void intake();
void outtake(double volts);
void outtake();

// ================ UTILS ================

void robot_init();