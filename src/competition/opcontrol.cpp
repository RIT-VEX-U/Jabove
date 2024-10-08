#include "competition/opcontrol.h"
#include "automation.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "tuning.h"
#include "vex.h"

#define LINK_BUTTON_AND_DIGOUT(but, sol) but.pressed([]() { sol.set(!sol.value()); })

std::atomic<bool> drive(true);
std::atomic<TankDrive::BrakeType> brake_type(TankDrive::BrakeType::None);

// Button definitions
const vex::controller::button &intake_button = con.ButtonR1;
const vex::controller::button &outtake_button = con.ButtonR2;

const vex::controller::button &left_wing_buttom = con.ButtonDown;
const vex::controller::button &right_wing_button = con.ButtonB;

const vex::controller::button &both_wing_button = con.ButtonL1;
const vex::controller::button &brake_mode_button = con.ButtonL2;

const vex::controller::button &rclimb_wing_button = con.ButtonRight;
const vex::controller::button &back_wing_button = con.ButtonY;

const vex::controller::button &spin_button = con.ButtonUp;

const double spin_speed = 0.45;

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {

  // ================ INIT ================
  while (imu.isCalibrating()) {
    vexDelay(1);
  }

  LINK_BUTTON_AND_DIGOUT(left_wing_buttom, left_wing_sol);
  LINK_BUTTON_AND_DIGOUT(right_wing_button, right_wing_sol);

  LINK_BUTTON_AND_DIGOUT(rclimb_wing_button, rclimb_wing_sol);
  LINK_BUTTON_AND_DIGOUT(back_wing_button, back_wing_sol);

  LINK_BUTTON_AND_DIGOUT(both_wing_button, left_wing_sol);
  LINK_BUTTON_AND_DIGOUT(both_wing_button, right_wing_sol);
  LINK_BUTTON_AND_DIGOUT(con.ButtonY, vision_light);

  // intake
  intake_button.pressed([]() { intake(); });
  outtake_button.pressed([]() { outtake(); });
  brake_mode_button.pressed([]() {
    if (brake_type == TankDrive::BrakeType::None) {
      brake_type = TankDrive::BrakeType::Smart;
    } else {
      brake_type = TankDrive::BrakeType::None;
    }
  });

  spin_button.pressed([]() { drive = false; });
  spin_button.released([]() { drive = true; });

  // ================ PERIODIC ================
  while (true) {
    // fflush(stdout);
    if (drive) {
      if (!intake_button.pressing() && !outtake_button.pressing()) {
        intake_motors.stop(vex::brakeType::hold);
      }

      double straight = (double)con.Axis3.position() / 100;
      double turn = (double)con.Axis1.position() / 100;

      drive_sys.drive_arcade(straight, turn * 0.75, 1, brake_type);
      vexDelay(10);
    } else if (spin_button.pressing()) {
      drive_sys.drive_tank_raw(-spin_speed, spin_speed);
      vexDelay(10);
    } else {
      vexDelay(10);
    }
  }
}

void testing() {
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
  while (true) {
    tune_drive_pid(DriveType::DRIVE);
    // tune_drive_ff_kv(DRIVE, 0.05);
    // tune_drive_motion_maxv(DriveType::DRIVE);
    vexDelay(20);
  }

  LINK_BUTTON_AND_DIGOUT(con.ButtonY, vision_light);

  con.ButtonA.pressed([]() {
    const double test_distance = 80;

    odom.set_position({.x = 0, .y = 0, .rot = 90});

    CommandController cc{
      drive_sys.DriveForwardCmd(test_distance, vex::directionType::fwd)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.TurnToHeadingCmd(270, 0.3)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(test_distance, vex::fwd)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.TurnToHeadingCmd(90, 0.3)->withTimeout(2.0),
    };
    cc.add_cancel_func([]() { return con.ButtonX.pressing(); });
    cc.run();
  });

  con.ButtonB.pressed([]() {
    const double test_distance = 40;

    odom.set_position({.x = 0, .y = 0, .rot = 90});

    CommandController cc{
      drive_sys.DriveForwardCmd(test_distance, vex::directionType::fwd, 1)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.TurnToHeadingCmd(270, 0.3)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.DriveForwardCmd(test_distance, vex::fwd, 1)->withTimeout(2.0),
      new DelayCommand(500),
      drive_sys.TurnToHeadingCmd(90, 0.3)->withTimeout(2.0),
    };
    cc.add_cancel_func([]() { return con.ButtonX.pressing(); });
    cc.run();
  });

  while (1) {
    auto pos = odom.get_position();
    printf("X: %.2f, Y: %.2f, rot: %.2f\n", pos.x, pos.y, pos.rot);
    // tune_drive_pid(DRIVE);
    vexDelay(20);
  }
}