#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "landscape_opt/indices/eca.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

int main(int argc, const char * argv[]) {
    std::filesystem::path instances_description_json = argv[1];

    Instance instance = parse_instance(instances_description_json);

    melon::

    return EXIT_SUCCESS;
}
