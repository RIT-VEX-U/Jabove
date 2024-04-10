#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

std::atomic<bool> drive(true);

// Button definitions
const vex::controller::button &intake_button = con.ButtonR2;
const vex::controller::button &outtake_button = con.ButtonR1;

const vex::controller::button &climb_wing_button = con.ButtonL2;

const vex::controller::button &right_wing_button = con.ButtonB;
const vex::controller::button &left_wing_buttom = con.ButtonDown;
const vex::controller::button &light_button = con.ButtonY;

#define LINK_BUTTON_AND_DIGOUT(but, sol) but.pressed([]() { sol.set(!sol.value()); })

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
  LINK_BUTTON_AND_DIGOUT(climb_wing_button, climb_wing_sol);
  LINK_BUTTON_AND_DIGOUT(climb_wing_button, climb_wing_sol);
  LINK_BUTTON_AND_DIGOUT(light_button, headlights);

  // intake
  intake_button.pressed([]() { intake(); });
  outtake_button.pressed([]() { outtake(); });

  con.ButtonUp.pressed([]() { drive = !drive; });

  //   con.ButtonUp.pressed([]() { odom.set_position({.x = 0, .y = 0, .rot = 0}); });

  con.ButtonA.pressed([]() {
    drive = false;
    CommandController cc{
      drive_sys.DriveForwardCmd(24.0, vex::fwd),
    };
    cc.run();
    drive = true;
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

      drive_sys.drive_arcade(straight, turn * 0.75, 1, TankDrive::BrakeType::None);
      vexDelay(10);
    } else {
      vexDelay(10);
    }
  }
}