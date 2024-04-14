#include "competition/opcontrol.h"
#include "automation.h"
#include "robot-config.h"
#include "tuning.h"
#include "vex.h"

#define SOFTTEST
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

const vex::controller::button &climb_wing_button = con.ButtonRight;

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
  testing();
  // ================ INIT ================
  while (imu.isCalibrating()) {
    vexDelay(1);
  }

#ifdef SOFTTEST
  con.ButtonUp.pressed([]() { drive = !drive; });

  con.ButtonLeft.pressed([]() { odom.set_position({.x = 0, .y = 0, .rot = 0}); });

  // con.ButtonA.pressed([]() {
  //   CommandController cc{
  //     // drive_sys.DriveForwardCmd(drive_pid, 48.0, vex::fwd),
  //     drive_sys.TurnDegreesCmd(90),
  //     // new IntakeToHold()
  //   };
  //   cc.add_cancel_func([](){return con.ButtonX.pressing();});
  //   cc.run();
  // });

  LINK_BUTTON_AND_DIGOUT(con.ButtonY, vision_light);
#endif

  LINK_BUTTON_AND_DIGOUT(left_wing_buttom, left_wing_sol);
  LINK_BUTTON_AND_DIGOUT(right_wing_button, right_wing_sol);
  LINK_BUTTON_AND_DIGOUT(climb_wing_button, climb_wing_sol);
  LINK_BUTTON_AND_DIGOUT(both_wing_button, left_wing_sol);
  LINK_BUTTON_AND_DIGOUT(both_wing_button, right_wing_sol);

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

  // ================ PERIODIC ================
  while (true) {
    fflush(stdout);
    if (drive) {
      if (!intake_button.pressing() && !outtake_button.pressing()) {
        intake_motors.stop(vex::brakeType::hold);
      }

      double straight = (double)con.Axis3.position() / 100;
      double turn = (double)con.Axis1.position() / 100;

      drive_sys.drive_arcade(straight, turn * 0.75, 1, brake_type);
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
  while (1) {
    tune_drive_pid(DRIVE);
    vexDelay(20);
  }
}