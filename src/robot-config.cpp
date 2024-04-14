#include "robot-config.h"
#include "../core/include/subsystems/fun/video.h"

#define FWD vex::fwd
#define REV vex::reverse

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT19, vex::turnType::right);

// Analog sensors
const double intake_sensor_dist_mm = 50;
vex::distance intake_sensor(vex::PORT21);

// ================ OUTPUTS ================
// Climb Motors
vex::motor left_climb(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::motor right_climb(vex::PORT14, vex::gearSetting::ratio36_1, false);

// Intake Motors
vex::motor left_intake(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_intake(vex::PORT10, vex::gearSetting::ratio6_1, false);

// Drivetrain motors
vex::motor left_front(vex::PORT2, vex::gearSetting::ratio6_1, true);
vex::motor left_second(vex::PORT14, vex::gearSetting::ratio6_1, true);
vex::motor left_third(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor left_back(vex::PORT11, vex::gearSetting::ratio6_1, true);

vex::motor right_front(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor right_second(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_third(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor right_back(vex::PORT20, vex::gearSetting::ratio6_1, false);

std::map<std::string, vex::motor &> motor_names{
  {"left 1", left_front},   {"left 2", left_second},   {"left 3", left_third},   {"left 4", left_back},

  {"right 1", right_front}, {"right 2", right_second}, {"right 3", right_third}, {"right 4", right_back},

};

vex::motor_group left_motors = {left_front, left_second, left_third, left_back};
vex::motor_group right_motors = {right_front, right_second, right_third, right_back};

vex::motor_group intake_motors = {left_intake, right_intake};

CustomEncoder center_enc{Brain.ThreeWirePort.E, 2048};

vex::digital_out left_wing_sol{Brain.ThreeWirePort.B};
vex::digital_out right_wing_sol{Brain.ThreeWirePort.G};
vex::digital_out climb_wing_sol{Brain.ThreeWirePort.A};
vex::digital_out vision_light{Brain.ThreeWirePort.D};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.1,
  .i = 0.0,
  .d = 0.005,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t drive_correction_pid{
  .p = 0.00,
  .i = 0.0,
  .d = 0.0,
  .deadband = 0.0,
};

PID::pid_config_t turn_pid_cfg{
  .p = 0.001, 
  .i = 0.0, 
  .d = 0.0, 
  .deadband = 0.0, 
  .on_target_time = 0.0
};

PID turn_pid{turn_pid_cfg};

PID::pid_config_t drive_mc_pid_cfg{
  .p = 0.0,
  .i = 0.0,
  .d = 0.0,
  .deadband = 0.0,
  .on_target_time = 0.0,
};

FeedForward::ff_config_t drive_mc_ff_cfg{
  .kS = 0.05,
  .kV = 0.012,
  .kA = 0.001,
};

MotionController::m_profile_cfg_t drive_mc_fast_cfg{
  .max_v = 50,
  .accel = 50,
  .pid_cfg = drive_mc_pid_cfg,
  .ff_cfg = drive_mc_ff_cfg,
};

MotionController drive_mc_fast(drive_mc_fast_cfg);

PID::pid_config_t turn_mc_pid_cfg{
  .p = 0.0,
  .i = 0.0,
  .d = 0.0,
  .deadband = 0.0,
  .on_target_time = 0.0,
};

FeedForward::ff_config_t turn_mc_ff_cfg{
  .kS = 0.0,
  .kV = 0.0,
  .kA = 0.0,
};

MotionController::m_profile_cfg_t turn_mc_fast_cfg{
  .max_v = 0,
  .accel = 0,
  .pid_cfg = turn_mc_pid_cfg,
  .ff_cfg = turn_mc_ff_cfg,
};

MotionController turn_mc_fast(turn_mc_fast_cfg);


// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
  .robot_radius = 12.0,
  .odom_wheel_diam = 2.0,
  .odom_gear_ratio = 1.0,
  .dist_between_wheels = 10.0,

  .drive_correction_cutoff = 8.0,
  .drive_feedback = &drive_mc_fast,
  .turn_feedback = &turn_mc_fast,
  .correction_pid = drive_correction_pid,
};

OdometryTank odom(center_enc, center_enc, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init() {
  imu.startCalibration();

  // set video
  set_video("funkylow.mpeg");
  screen::start_screen(
    Brain.Screen,
    {
      new VideoPlayer(),
      new screen::PIDPage(drive_pid, "drive"),
      new screen::OdometryPage(odom, 12.1, 14.95, true),
      new screen::StatsPage(motor_names),
    },
    1
  );
}