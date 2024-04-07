#include "robot-config.h"
#include "../core/include/subsystems/fun/video.h"

#define FWD vex::fwd
#define REV vex::reverse


vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT15, vex::turnType::right);

// Analog sensors

// ================ OUTPUTS ================
// Climb Motors 
vex::motor left_climb(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::motor right_climb(vex::PORT14, vex::gearSetting::ratio36_1, false);

// Intake Motors
vex::motor left_intake(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_intake(vex::PORT10, vex::gearSetting::ratio6_1, false);

// Drivetrain motors
vex::motor left_front(vex::PORT2, vex::gearSetting::ratio6_1, true);
vex::motor left_second(vex::PORT5, vex::gearSetting::ratio6_1, true);
vex::motor left_third(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor left_back(vex::PORT11, vex::gearSetting::ratio6_1, true);

vex::motor right_front(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor right_second(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_third(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor right_back(vex::PORT20, vex::gearSetting::ratio6_1, false);

vex::motor_group left_motors = {left_front, left_second, left_third, left_back};
vex::motor_group right_motors = {right_front, right_second, right_third, right_back};

vex::motor_group intake_motors = {left_intake, right_intake};

vex::motor_group climb_motors = {left_climb, right_climb};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg {
    .p = 0.1,
    .i = 0.0,
    .d = 0.005,
    .deadband = 1.0,
    .on_target_time = 0.5,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t drive_correction_pid {
    .p = 0.15,
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
    .robot_radius = 12.0,
    .odom_wheel_diam = 2.75,
    .odom_gear_ratio = 23.0 / 16.0,
    .dist_between_wheels = 10.0,

    .drive_correction_cutoff = 8.0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = drive_correction_pid,
};

OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

const double intake_volts = 10.0;
void intake(double volts) { intake_motors.spin(FWD, volts, vex::volt); };
void intake() { intake_motors.spin(FWD, intake_volts, vex::volt); };

void outtake(double volts) { intake_motors.spin(REV, volts, vex::volt); };
void outtake() { intake_motors.spin(REV, intake_volts, vex::volt); };

const double climb_volts = 12.0;
void out_climb(double volts) { climb_motors.spin(FWD, volts, vex::volt); };
void out_climb() { climb_motors.spin(FWD, climb_volts, vex::volt); };

void climb_up(double volts) { climb_motors.spin(REV, volts, vex::volt); };
void climb_up() { climb_motors.spin(REV, climb_volts, vex::volt); };

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();

    // set video
    set_video("funkylow.mpeg");
    screen::start_screen(
        Brain.Screen,
        {
            new VideoPlayer(), new screen::PIDPage(drive_pid, "drive"), new screen::OdometryPage(odom, 12.1, 14.95, true)
        },
        1
    );

}