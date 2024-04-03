#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT1, vex::turnType::right);

// Analog sensors

// ================ OUTPUTS ================
// Intake Motors
vex::motor left_intake(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor right_intake(vex::PORT10, vex::gearSetting::ratio6_1, false);

// Drivetrain motors
vex::motor left_front(vex::PORT2, vex::gearSetting::ratio6_1, false);
vex::motor left_second(vex::PORT5, vex::gearSetting::ratio6_1, false);
vex::motor left_third(vex::PORT4, vex::gearSetting::ratio6_1, false);
vex::motor left_back(vex::PORT11, vex::gearSetting::ratio6_1, false);

vex::motor right_front(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor right_second(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_third(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor right_back(vex::PORT20, vex::gearSetting::ratio6_1, false);

vex::motor_group left_motors = {left_front, left_second, left_third, left_back};
vex::motor_group right_motors = {right_front, right_second, right_third, right_back};

vex::motor_group intake_motors = {left_intake, right_intake};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg {
    .p = 0.0,
    .i = 0.0,
    .d = 0.0,
    .deadband = 0.0,
    .on_target_time = 0.0,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t drive_correction_pid {
    .p = 0.0,
    .i = 0.0,
    .d = 0.0,
    .deadband = 0.0,
};

PID::pid_config_t turn_pid_cfg {
    .p = 0.0,
    .i = 0.0,
    .d = 0.0,
    .deadband = 0.0,
    .on_target_time = 0.0
};

PID turn_pid{turn_pid_cfg};

// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
    .robot_radius = 0.0,
    .odom_wheel_diam = 0.0,
    .odom_gear_ratio = 0.0,
    .dist_between_wheels = 0.0,

    .drive_correction_cutoff = 0.0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = drive_correction_pid,
};

OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();
}