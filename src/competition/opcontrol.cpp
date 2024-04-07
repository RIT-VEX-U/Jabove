#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

std::atomic<bool> drive(true);

// Button definitions
const vex::controller::button &intake_button = con.ButtonR2;
const vex::controller::button &outtake_button = con.ButtonR1;

const vex::controller::button &shoot_climb_button = con.ButtonL1;
const vex::controller::button &climb_up_button = con.ButtonL2;

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================
    while (imu.isCalibrating()) {
        vexDelay(1);
    }

    // intake
    intake_button.pressed([]() { intake(); } );
    outtake_button.pressed([]() { outtake(); });

    shoot_climb_button.pressed([]() { out_climb(); } );
    climb_up_button.pressed([]() { climb_up(); });

    con.ButtonDown.pressed([]() {

        CommandController cc{
            new FunctionCommand([]() {
                printf(">>> START POS: X = %f, Y = %f, rot = %f\n", odom.get_position().x, odom.get_position().y, odom.get_position().rot);
                return true;
            }),

            drive_sys.DriveForwardCmd(drive_pid, 4 * 12, vex::fwd, 0.75)->withTimeout(5),

            new FunctionCommand([]() {
                printf(">>> END POS: X = %f, Y = %f, rot = %f\n", odom.get_position().x, odom.get_position().y, odom.get_position().rot);
                return true;
            })
        };
        
        cc.run();
    });

    con.ButtonUp.pressed([]() { drive = !drive; });

    con.ButtonLeft.pressed([]() { odom.set_position({.x=0, .y=0, .rot=0}); });

    // ================ PERIODIC ================
    while (true) {
        if (drive) {
            if (!intake_button.pressing() && !outtake_button.pressing()) {
                intake_motors.stop(vex::brakeType::hold);
            }

            if (!shoot_climb_button.pressing() && !climb_up_button.pressing()) {
                climb_motors.stop(vex::brakeType::hold);
            }

            double straight = (double)con.Axis3.position() / 100;
            double turn = (double)con.Axis1.position() / 100;

            drive_sys.drive_arcade(straight, turn * 0.75 , 1, TankDrive::BrakeType::None);
            vexDelay(10);
        } else {
            vexDelay(10);
        }
    }

}