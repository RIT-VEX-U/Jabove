#include "competition/autonomous.h"
#include "robot-config.h"
#include "automation.h"

/**
 * Main entrypoint for the autonomous period
 */
void just_auto();

void autonomous()
{
    while (imu.isCalibrating()) {
        vexDelay(1);
    }

    just_auto();
}

AutoCommand *intake_cmd(double amt = 8.0) {
  return new FunctionCommand([=]() {
    intake(amt);
    return true;
  });
}
AutoCommand *outtake_cmd(double amt = 8.0) {
  return new FunctionCommand([=]() {
    outtake(amt);
    return true;
  });
}

class DebugCommand : public AutoCommand {
public:
  bool run() override {
    drive_sys.stop();
    pose_t pos = odom.get_position();
    printf("ODO X: %.2f, Y: %.2f, R:%.2f, ", pos.x, pos.y, pos.rot);
    while (true) {
      double f = con.Axis3.position() / 200.0;
      double s = con.Axis1.position() / 200.0;
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
      pose_t pos = odom.get_position();
      printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
      // printf("GPS X: %.2f, Y: %.2f, R: %.2f Q: %d\n",
      //     gps_sensor.xPosition(distanceUnits::in)+72,
      //     gps_sensor.yPosition(distanceUnits::in)+72,
      //     gps_sensor.heading(), gps_sensor.quality());
      //   );
      vexDelay(100);
    }
    return false;
  }
};
AutoCommand *toggle_wing_r() {
  return new FunctionCommand([]() {
    right_wing_sol.set(!right_wing_sol);
    return true;
  });
}

AutoCommand *toggle_wing_l() {
  return new FunctionCommand([]() {
    left_wing_sol.set(!left_wing_sol);
    return true;
  });
}

AutoCommand *stop_intake() {
  return new FunctionCommand([]() {
    intake(0);
    return true;
  });
}

void just_auto() {
  // clang-format off
  CommandController cc{
    odom.SetPositionCmd({.x = 40, .y = 7, .rot = 90}),     
    drive_sys.DriveToPointCmd({40, 35}, vex::fwd, 0.75)->withTimeout(2.0),
    
    // pick up second
    drive_sys.TurnToPointCmd(47.2, 54.7, vex::fwd, 0.65)->withTimeout(2.0),
    intake_cmd(12.0),
    drive_sys.DriveToPointCmd({47.63, 54.5}, vex::fwd, 0.5)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({37.7, 34.5}, vex::reverse, 0.75)->withTimeout(2.0),
    stop_intake(),
    drive_sys.TurnToPointCmd(37.5, 20.0, vex::fwd, 0.65)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({37.5, 20.0}, vex::fwd, 0.75)->withTimeout(2.0),

    // Drop Second
    drive_sys.TurnToHeadingCmd(340, 0.65)->withTimeout(2.0),
    outtake_cmd(),
    new DelayCommand(600),
    stop_intake(),


    // get close to wall
    drive_sys.TurnToHeadingCmd(240, 0.65)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({33.5,12.5}, vex::fwd,0.25)->withTimeout(3.0),

    // turn and back up to bar
    drive_sys.TurnToPointCmd(22.0,16.0, vex::reverse,0.65)->withTimeout(5.0),
    drive_sys.DriveToPointCmd({21,16.0},vex::reverse,0.15)->withTimeout(5.0),
    new RepeatUntil({
        toggle_wing_r(), // out
        new DelayCommand(400),
        toggle_wing_r(), // in
        new DelayCommand(800),
    }, (new IfTimePassed(35))->Or(new TimesTestedCondition(10))),

    // pushing
    toggle_wing_l(),
    outtake_cmd(),
    drive_sys.DriveToPointCmd({37, 6.5},vex::fwd,0.5)->withTimeout(3.0),
    drive_sys.TurnToHeadingCmd(-10, 0.5)->withTimeout(3.0),
  // stop things circa half court
    new Async(new FunctionCommand([](){
      if (odom.get_position().x > 55){
        left_wing_sol.set(false);
      }
      if (odom.get_position().x > 69){
        left_wing_sol.set(true);
        outtake(0);
        return true;
      }
      return false;
    })),

    // across half court
    drive_sys.DriveToPointCmd({108, 5},vex::fwd, 0.75)->withTimeout(3.0),

    drive_sys.TurnToPointCmd(122.5,16.52,vex::fwd, 0.5)->withTimeout(1.0),
    drive_sys.DriveToPointCmd({122.5, 16.52},vex::fwd, 0.5)->withTimeout(3.0),

    toggle_wing_l(),
    drive_sys.TurnDegreesCmd(-180)->withTimeout(0.35), // start spinning the right way

    drive_sys.TurnToPointCmd(132.5,36.2, vex::reverse, 0.5)->withTimeout(3.0),
    drive_sys.DriveForwardCmd(25, vex::reverse, 1.0)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(27, vex::fwd, 1.0)->withTimeout(1.5),

    // drive_sys.TurnToHeadingCmd(247, 0.65)->withTimeout(3.0),
    // drive_sys.DriveForwardCmd(27, vex::reverse, 1.0)->withTimeout(1.5),
    // drive_sys.DriveForwardCmd(20, vex::fwd, 1.0)->withTimeout(1.5),
// 
    // drive_sys.TurnToHeadingCmd(227, 0.65)->withTimeout(3.0),
    // drive_sys.DriveForwardCmd(20, vex::reverse, 1.0)->withTimeout(1.5),
    // drive_sys.DriveForwardCmd(27, vex::fwd, 1.0)->withTimeout(1.5),

    drive_sys.TurnToHeadingCmd(90, 0.5)->withTimeout(2.0),
    drive_sys.DriveTankCmd(-0.25,-0.25)->withTimeout(2.0),
    drive_sys.DriveForwardCmd(2.0,vex::fwd, 0.5)->withTimeout(3.0),

    drive_sys.TurnToHeadingCmd(180, 0.5)->withTimeout(1.0),
    drive_sys.DriveForwardCmd(38.0,vex::fwd, 0.5)->withTimeout(3.0),
    drive_sys.TurnToHeadingCmd(70, 0.5)->withTimeout(3.0),
    intake_cmd(12.0),
    drive_sys.DriveForwardCmd(3.0, vex::fwd, 0.15)->withTimeout(4.0),
    new DelayCommand(500),
    intake_cmd(0),
    // new DebugCommand(),
  };
    cc.run();
  // clang-format on
}