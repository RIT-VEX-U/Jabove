#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================
    while (imu.isCalibrating()) {
        vexDelay(1);
    }



    // ================ PERIODIC ================
    while (true) {
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis1.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);
    }

}