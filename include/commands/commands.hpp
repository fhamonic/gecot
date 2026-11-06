#ifndef COMMANDS_HPP
#define COMMANDS_HPP

#include <string>
#include <vector>

// args: the command line after the command name; returns the exit code
int optimize_command(const std::vector<std::string> & args);
int bound_command(const std::vector<std::string> & args);

#endif  // COMMANDS_HPP
