#include "../core/include/utils/command_structure/message_command.h"


MessageCommand::MessageCommand(std::function<std::string()> f) : f(f) {};

bool MessageCommand::run(){
    std::string str = f();

    printf("%s\n", str.c_str());

    return true;
};