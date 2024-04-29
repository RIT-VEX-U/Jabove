#include "competition/autonomous.h"
#include "automation.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */
void just_auto();
void elims_auto();
void skills();

void autonomous() {
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
  skills();

  //   skills();
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
  // intake_motors.setMaxTorque()
  // clang-format off
  CommandController cc{


    odom.SetPositionCmd({.x = 40, .y = 7, .rot = 90}),     

    outtake_cmd(),
    new DelayCommand(200),

    intake_cmd(10.0),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {40.0, 7.0},
        {40.0, 29.74},
        {47,	53.0},
    }, 2.0), vex::fwd, 0.4)->withTimeout(3.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    new DelayCommand(300),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {48,	54.0},
        {40.0, 31.34},
        {40.0, 17.0},
    }, 2.0), vex::reverse, 0.4)->withTimeout(3.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),
    // drive_sys.DriveToPointCmd({40, 35}, vex::fwd, 0.75)->withTimeout(2.0),    
    // pick up field triball
    // drive_sys.TurnToPointCmd(49.2, 56.7, vex::fwd, 0.65)->withTimeout(2.0),
    // drive_sys.DriveToPointCmd({49.0, 56.0}, vex::fwd, 0.45)->withTimeout(3.0),


    // Drop triball off
    drive_sys.TurnToHeadingCmd(340, 0.65)->withTimeout(1.5),
    outtake_cmd(),
    new DelayCommand(450),
    stop_intake(),


    // get close to wall
    drive_sys.TurnToHeadingCmd(240, 0.65)->withTimeout(1.0), 


    drive_sys.DriveToPointCmd({37.8, 11.20}, vex::fwd, 0.3)->withTimeout(2.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.05)),

    // turn and back up to bar
    drive_sys.TurnToHeadingCmd(343, 0.5)->withTimeout(2.0),

    // new DebugCommand(),
    // new FunctionCommand([]() {
        // auto pos = odom.get_position();
        // printf("X: %f, Y: %f, rot: %f\n", pos.x, pos.y, pos.rot);
        // return true;
    // }),

    drive_sys.DriveToPointCmd({21.5, 16.8}, vex::reverse, 0.25)->withTimeout(1.5), // (22.7, 16) - 340

    // new FunctionCommand([]() {
        // auto pos = odom.get_position();
        // printf("X: %f, Y: %f, rot: %f\n", pos.x, pos.y, pos.rot);
        // return true;
    // }),


    new DelayCommand(600),
    // match load
    new RepeatUntil({
        toggle_wing_r(), // out
        new DelayCommand(300),
        toggle_wing_r(), // in
        new DelayCommand(600),
    }, (new IfTimePassed(35))->Or(new TimesTestedCondition(10))),


    new Async(new InOrder{
        new WaitUntilCondition(new FunctionCondition([](){
            return odom.get_position().x > 105;
        })),
        toggle_wing_r(),
    }),
    // pushing
    toggle_wing_l(),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {21.63,	-3 + 16.3},
        {33.04,	-2 + 9.91},
        {44.75,	-1 + 6.5},
        {76,	5.3},
        {110,	 6.4},
        {120,	 14.25},
        {132,	 27.75},
        // {121.5,	-1 + 31.5},
    }, 8.0), vex::fwd, 0.3)->withTimeout(9.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    toggle_wing_l(),

    // drive_sys.DriveForwardCommand(7.5, vex::reverse, 0.4)->withTimeout(2.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    // make sure wings are in
    new FunctionCommand([]() {
      left_wing_sol.set(false);
      right_wing_sol.set(false);
      return true;
    }),

    drive_sys.TurnDegreesCmd(90)->withTimeout(0.2),

    new DelayCommand(300),

    drive_sys.TurnToHeadingCmd(-90)->withTimeout(1.0),
    outtake_cmd(),

    new FunctionCommand([]() {
        back_wing_sol.set(true);
        return true;
    }),

    // push once
    // new DebugCommand(),
    drive_sys.DriveForwardCmd(18, vex::reverse, 1.0)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({128.4, 28.26}, vex::forward, 0.6)->withTimeout(2.0),
    drive_sys.TurnToHeadingCmd(-110)->withTimeout(2.0),

    //push again
    drive_sys.DriveForwardCmd(22, vex::reverse, 1.0)->withTimeout(1.5),

    drive_sys.TurnToHeadingCmd(-90)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({118.4, 7.26}, vex::forward, 0.6)->withTimeout(2.0),

    new FunctionCommand([]() {
        back_wing_sol.set(false);
        return true;
    }),

    new DelayCommand(300),

    // new DebugCommand(),

    drive_sys.TurnToPointCmd(74.4, 6.86, vex::forward, 0.5)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({74.4, 6.86}, vex::forward, 0.6)->withTimeout(2.0),


    stop_intake(),

    drive_sys.TurnToHeadingCmd(90, 0.5)->withTimeout(2.0),
    drive_sys.DriveForwardCmd(4, vex::forward, 0.1)->withCancelCondition(drive_sys.DriveStalledCondition(0.2))->withTimeout(5.0),

  };
    cc.run();
  // clang-format on
}

void elims_auto() {
  // intake_motors.setMaxTorque()
  // clang-format off
  CommandController cc{


    odom.SetPositionCmd({.x = 40, .y = 7, .rot = 90}),     

    outtake_cmd(),
    new DelayCommand(200),

    intake_cmd(10.0),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {40.0, 7.0},
        {40.0, 29.74},
        {47,	53.0},
    }, 2.0), vex::fwd, 0.4)->withTimeout(3.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    new DelayCommand(300),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {48,	54.0},
        {40.0, 31.34},
        {40.0, 17.0},
    }, 2.0), vex::reverse, 0.4)->withTimeout(3.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),
    // drive_sys.DriveToPointCmd({40, 35}, vex::fwd, 0.75)->withTimeout(2.0),    
    // pick up field triball
    // drive_sys.TurnToPointCmd(49.2, 56.7, vex::fwd, 0.65)->withTimeout(2.0),
    // drive_sys.DriveToPointCmd({49.0, 56.0}, vex::fwd, 0.45)->withTimeout(3.0),


    // Drop triball off
    drive_sys.TurnToHeadingCmd(340, 0.65)->withTimeout(1.5),
    outtake_cmd(),
    new DelayCommand(450),
    stop_intake(),


    // get close to wall
    drive_sys.TurnToHeadingCmd(240, 0.65)->withTimeout(1.0), 


    drive_sys.DriveToPointCmd({37.8, 11.20}, vex::fwd, 0.3)->withTimeout(2.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.05)),

    // turn and back up to bar
    drive_sys.TurnToHeadingCmd(343, 0.5)->withTimeout(2.0),

    // new DebugCommand(),
    // new FunctionCommand([]() {
        // auto pos = odom.get_position();
        // printf("X: %f, Y: %f, rot: %f\n", pos.x, pos.y, pos.rot);
        // return true;
    // }),

    drive_sys.DriveToPointCmd({21.5, 16.8}, vex::reverse, 0.25)->withTimeout(1.5), // (22.7, 16) - 340

    // new FunctionCommand([]() {
        // auto pos = odom.get_position();
        // printf("X: %f, Y: %f, rot: %f\n", pos.x, pos.y, pos.rot);
        // return true;
    // }),


    new DelayCommand(600),
    // match load
    new RepeatUntil({
        toggle_wing_r(), // out
        new DelayCommand(300),
        toggle_wing_r(), // in
        new DelayCommand(600),
    }, (new IfTimePassed(35))->Or(new TimesTestedCondition(10))),


    new Async(new InOrder{
        new WaitUntilCondition(new FunctionCondition([](){
            return odom.get_position().x > 105;
        })),
        toggle_wing_r(),
    }),
    // pushing
    toggle_wing_l(),
    drive_sys.PurePursuitCmd(PurePursuit::Path({
        {21.63,	-3 + 16.3},
        {33.04,	-2 + 9.91},
        {44.75,	-1 + 6.5},
        {76,	5.3},
        {110,	 6.4},
        {120,	 14.25},
        {132,	 27.75},
        // {121.5,	-1 + 31.5},
    }, 8.0), vex::fwd, 0.5)->withTimeout(5.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    toggle_wing_l(),

    // make sure wings are in
    new FunctionCommand([]() {
      left_wing_sol.set(false);
      right_wing_sol.set(false);
      return true;
    }),

    drive_sys.TurnDegreesCmd(90)->withTimeout(0.2),

    new DelayCommand(300),

    drive_sys.TurnToHeadingCmd(-90)->withTimeout(1.0),
    outtake_cmd(),


    // push once
    // new DebugCommand(),
    drive_sys.DriveForwardCmd(18, vex::reverse, 1.0)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({128.4, 28.26}, vex::forward, 0.6)->withTimeout(2.0),
    drive_sys.TurnToHeadingCmd(-110)->withTimeout(2.0),

    //push again
    drive_sys.DriveForwardCmd(22, vex::reverse, 1.0)->withTimeout(1.5),

    drive_sys.TurnToHeadingCmd(-90)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({118.4, 7.26}, vex::forward, 0.6)->withTimeout(2.0),

    new DelayCommand(300),

    // new DebugCommand(),

    drive_sys.TurnToPointCmd(74.4, 6.86, vex::forward, 0.5)->withTimeout(2.0),
    drive_sys.DriveToPointCmd({74.4, 6.86}, vex::forward, 0.6)->withTimeout(2.0),


    drive_sys.TurnToHeadingCmd(180),
    drive_sys.DriveForwardCmd(20, vex::forward, 0.6)->withTimeout(2.0)->withCancelCondition(drive_sys.DriveStalledCondition(0.3)),

    stop_intake(),

  };
    cc.run();
  // clang-format on
}

AutoCommand *setBrakeHold() {
  return new FunctionCommand([]() {
    left_motors.setStopping(vex::brakeType::hold);
    right_motors.setStopping(vex::brakeType::hold);
    return true;
  });
}

AutoCommand *setBrakeCoast() {
  return new FunctionCommand([]() {
    left_motors.setStopping(vex::brakeType::coast);
    right_motors.setStopping(vex::brakeType::coast);
    return true;
  });
}

void skills() {
  // clang-format off

  CommandController cc{
    odom.SetPositionCmd({.x = 25.08, .y = 14.86, .rot = 334}),

    outtake_cmd(),
    new DelayCommand(200),

    // new DebugCommand(),

    // new DebugCommand(),
    setBrakeHold(),

    new RepeatUntil(
      {
        toggle_wing_r(),
        new DelayCommand(400),
        toggle_wing_r(),
        new DelayCommand(800),
      },
      (new IfTimePassed(35))->Or(new TimesTestedCondition(22))
    ),

    // new DebugCommand(),

    outtake_cmd(),

    setBrakeCoast(),

    // new Async(new InOrder{
    //     new WaitUntilCondition(new FunctionCondition([](){
    //         return odom.get_position().x > 110;
    //     })),
    //     toggle_wing_r(),
    //     new WaitUntilCondition(new FunctionCondition([](){
    //         return odom.get_position().x > 130;
    //     })),
    //     toggle_wing_r(),
    // }),
    // pushing
    toggle_wing_l(),
    drive_sys
      .PurePursuitCmd(
        PurePursuit::Path(
          {
            {21.63, 10.3}, {33.04, 6.4}, {44.75, 5.2}, {76, 4.2}, {90, 4.2}, {105, 5.2}, {122, 9.25}, {134, 23.75},
            // {121.5,	-1 + 31.5},
          },
          8.0
        ),
        vex::fwd, 0.5
      )
      ->withTimeout(5.0)
      ->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    // outtake_cmd(0),

    // new DebugCommand(),

    toggle_wing_l(),

    // make sure wings are in
    new FunctionCommand([]() {
      left_wing_sol.set(false);
      right_wing_sol.set(false);
      return true;
    }),

    // turn around
    drive_sys.TurnDegreesCmd(90)->withTimeout(0.2),
    new DelayCommand(300),
    drive_sys.TurnToHeadingCmd(-90)->withTimeout(1.0),

    outtake_cmd(),

    // drive_sys.TurnToHeadingCmd(90),

    // push once
    // ODO X: 132.74, Y: 36.19, R:276.50
    drive_sys.DriveToPointCmd({132.74, 42.19}, vex::reverse, 1.0)->withTimeout(1.5),
    // drive_sys.DriveForwardCmd(18, vex::reverse, 1.0)->withTimeout(1.5),

    // ODO X: 133.41, Y: 24.83, R:272.45
    // drive_sys.DriveForwardCmd(17, vex::fwd, 1.0)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({133.42, 24.83}, vex::fwd, 0.7)->withTimeout(1.5),

    // push again (rework in fruture)
    // new DebugCommand(),
    // ODO X: 133.42, Y: 38.44, R:266.89
    drive_sys.DriveToPointCmd({133.42, 42.44}, vex::reverse, 1.0)->withTimeout(1.5),
    // drive_sys.DriveForwardCmd(16, vex::reverse, 1.0)->withTimeout(1.5),

    // drive_sys.DriveForwardCmd(17, vex::fwd, 1.0)->withTimeout(1.5),

    // new DebugCommand(),
    // drive_sys.DriveToPointCmd({128.4, 28.26}, vex::forward, 0.6)->withTimeout(2.0),
    // drive_sys.TurnToHeadingCmd(-110)->withTimeout(2.0),

    // push again
    //  drive_sys.DriveForwardCmd(18, vex::reverse, 1.0)->withTimeout(1.5),

    // drive_sys.TurnToHeadingCmd(-90)->withTimeout(2.0),

    // drive_sys.DriveForwardCmd(14, vex::fwd, 1.0)->withTimeout(1.5),
    // drive_sys.DriveToPointCmd({118.4, 7.26}, vex::forward, 0.6)->withTimeout(2.0),

    // new DelayCommand(150),

    // drive_sys.TurnToPointCmd(120.32, 14.81, vex::fwd, 0.8),
    // drive_sys.DriveToPointCmd({120.32, 14.81}, vex::fwd, 0.6),

    drive_sys
      .PurePursuitCmd(
        PurePursuit::Path(
          {
            {130.46, 26.56},
            {125.07, 18.81},
            {119.47, 13.10},
          },
          3.0
        ),
        vex::fwd, .35
      )
      ->withTimeout(3.5),
    new DelayCommand(200),

    outtake_cmd(0),

    // new DebugCommand(),
    drive_sys.TurnToPointCmd(92.51, 49.70, vex::fwd, 0.5)->withTimeout(1.0),

    toggle_wing_l(),
    toggle_wing_r(),

    drive_sys
      .PurePursuitCmd(
        PurePursuit::Path(
          {
            {119.78, 13.40}, {108.93, 32.0}, {100.78, 44.07}, {105.38, 54.8}, {118.78, 58.52},
            // {92.47, 13.10},
            // {91.09, 49.84},
            // {95.97, 64.14},
            // {89.84, 53.34},
            // {89.84, 61.0},
            //
            // front push
            // {108.14, 72.59},
            // {119.14, 72.59},
            // {125.53, 14.81},
            // {119.02, 11.23},
            // {115.91, 15.56},
            // {111.17, 25.05},
            // {100.62, 34.98},
            // {91.25, 60.15},
            // {106.37, 59.88},
          },
          2.0
        ),
        vex::fwd, 0.4
      )
      ->withTimeout(5.0)
      ->withCancelCondition(drive_sys.DriveStalledCondition(0.25)),

    toggle_wing_l(),
    // backup
    drive_sys.DriveToPointCmd({101.86, 77.38}, vex::reverse, 0.4)->withTimeout(3.0),

    drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(2.0),

    // second front push
    toggle_wing_l(),
    drive_sys.DriveForwardCmd(25, vex::fwd, 1.0)
      ->withTimeout(2.0)
      ->withCancelCondition(drive_sys.DriveStalledCondition(0.15)),

    drive_sys.DriveForwardCmd(25, vex::reverse, 1.0),

    outtake_cmd(0),
  };

  cc.run();

  // clang-format off
}