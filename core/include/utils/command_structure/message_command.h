/**
 *  File: message_controller.h
 *  Description: Interface for printing in auto
*/

#pragma once

#include "../core/include/utils/command_structure/auto_command.h"

/// Macro to send a string and have it print with the MessageCommand
#define MSGCMD(str) new MessageCommand([](){return (str);})


/**
 * Message Command is a command that takes a funcion that returns a 
 * string and prints it
*/
class MessageCommand : public AutoCommand{
    
    public:
        /// @brief Contruct a message command that print a particular msg
        /// @param f The string to be printed
        MessageCommand(std::function<std::string()> f);

        bool run();
    
    private:
        std::function<std::string()> f;
};

