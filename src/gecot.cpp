#include <cstdlib>
#include <print>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include "commands/commands.hpp"
#include "commands/common.hpp"

static void print_usage() {
    std::print(R"(Usage:
  gecot --help
  gecot --version
  gecot optimize --help
  gecot optimize <algorithm> <instance> <budget> [<parameters> ...]
  gecot bound --help
  gecot bound <method> <instance> <budget> [<parameters> ...]

Commands:
  optimize    Select options within a budget to maximize the score
  bound       Bound the best score of the options within a budget
)");
}

int main(int argc, const char * argv[]) {
#ifdef WIN32
    std::system("chcp 65001 > NUL");
#endif
    setup_logging();
    const std::vector<std::string> args(argv + 1, argv + argc);

    if(args.empty() || args[0] == "-h" || args[0] == "--help") {
        print_soft_name();
        print_usage();
        return EXIT_SUCCESS;
    }
    if(args[0] == "--version") {
        print_soft_name();
        return EXIT_SUCCESS;
    }
    const std::vector<std::string> command_args(args.begin() + 1, args.end());
    if(args[0] == "optimize") return optimize_command(command_args);
    if(args[0] == "bound") return bound_command(command_args);

    if(args[0].starts_with('-'))
        spdlog::error(
            "Parsing arguments: options go after the command, as in 'gecot "
            "optimize {0} ...' or 'gecot bound {0} ...'",
            args[0]);
    else
        spdlog::error(
            "Parsing arguments: unknown command '{}', see 'gecot --help'; "
            "'gecot_optimize <algorithm> ...' became 'gecot optimize "
            "<algorithm> ...'",
            args[0]);
    return EXIT_FAILURE;
}
