/**
 * File: debug_command.h
 * Desc:
 *      Use a DebugCommand to end a CommandController eary
 *      and print useful debug information
*/

#pragma once

#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

enum DebugInfo {ODOM, GPS};

class DebugCommand : public AutoCommand {
public:
    DebugCommand(TankDrive &drive_sys) : drive_sys(drive_sys), infoType(ODOM) {};
    bool run();

private:
    TankDrive &drive_sys;
    DebugInfo infoType;
};