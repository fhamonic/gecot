#ifndef MIP_SOLVER8WRAPPER
#define MIP_SOLVER8WRAPPER

#ifdef WIN32
#include <windows.h>
#endif

#include "mippp/solver_traits/all.hpp" 

std::filesystem::path get_exec_path() {
#ifdef WIN32
    char buffer[MAX_PATH];
    GetModuleFileName(NULL, buffer, MAX_PATH);
    return std::filesystem::path(buffer);
#else
    return std::filesystem::canonical("/proc/self/exe");
#endif
}

// mippp::cli_cbc_traits::exec_path = get_exec_path().parent_path() / "cbc.exe";

#endif // MIP_SOLVER8WRAPPER