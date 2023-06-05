#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

int main(int argc, const char * argv[]) {
    std::filesystem::path instances_description_json;

    Instance instance = parse_instance(instances_description_json);
    return EXIT_SUCCESS;
}
