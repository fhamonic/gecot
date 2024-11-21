#include <gtest/gtest.h>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "melon/mapping.hpp"
#include "melon/utility/graphviz_printer.hpp"
#include "melon/views/reverse.hpp"

#include "gecot/concepts/instance.hpp"
#include "gecot/indices/pc_num.hpp"

#include "optimize/instance.hpp"
#include "optimize/parse_instance.hpp"

using namespace fhamonic;

GTEST_TEST(preprocessing, fuzzy_test) {
    std::string instance_path = PROJECT_SOURCE_DIR;
    // instance_path.append("/test/instances/aude.json");
    instance_path.append("/test/instances/quebec_438_RASY.json");
    // instance_path.append("/test/instances/biorevaix_N1.json");
    Instance instance = parse_instance(instance_path);
    double budget = 0;
    for(auto && o : instance.options()) budget += instance.option_cost(o);
}
