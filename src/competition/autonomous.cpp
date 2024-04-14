#include "competition/autonomous.h"
#include "robot-config.h"
#include "automation.h"

/**
 * Main entrypoint for the autonomous period
 */
void autonomous()
{
    while (imu.isCalibrating()) {
        vexDelay(1);
    }
}