#include <gtest/gtest.h>
#include <string>

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

GTEST_TEST(preprocessing, fuzzy_test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    instance_path.append("/test/instances/aude.json");

    Instance instance = parse_instance(instance_path);
}
