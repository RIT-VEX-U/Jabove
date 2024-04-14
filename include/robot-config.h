#pragma once
#include "core.h"
#include "vex.h"

extern vex::brain Brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::inertial imu;
// Analog sensors
extern const double intake_sensor_dist_mm;
extern vex::distance intake_sensor;

// ================ OUTPUTS ================
// Motors
extern vex::motor_group intake_motors;
extern vex::motor_group left_motors;
extern vex::motor_group right_motors;
extern CustomEncoder center_enc;

// Pneumatics
extern vex::digital_out left_wing_sol;
extern vex::digital_out right_wing_sol;
extern vex::digital_out climb_wing_sol;
extern vex::digital_out vision_light;

extern vex::vision vision;

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;
extern PID drive_pid;

extern robot_specs_t robot_cfg;
extern MotionController drive_mc_fast;

// ================ UTILS ================

void robot_init();