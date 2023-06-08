#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "landscape_opt/indices/eca.hpp"
#include "landscape_opt/indices/parallel_eca.hpp"
#include "landscape_opt/utils/chronometer.hpp"

#include "landscape_opt/helper.hpp"
#include "landscape_opt/solvers/static_incremental.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"

using namespace fhamonic;

int main(int argc, const char * argv[]) {
    std::filesystem::path instances_description_json = argv[1];

    static_assert(landscape_opt::instance_c<Instance>);
    static_assert(landscape_opt::case_c<InstanceCase>);

    Instance instance = parse_instance(instances_description_json);
    InstanceCase instance_case = instance.cases().front();

    chronometer chrono;

    const double seq_eca = fhamonic::landscape_opt::eca(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "seq ECA = " << seq_eca << " in " << chrono.lap_time_us()
              << " us" << std::endl;

    const double par_eca = fhamonic::landscape_opt::parallel_eca(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "par ECA = " << par_eca << " in " << chrono.lap_time_us()
              << " us" << std::endl;

    landscape_opt::solvers::StaticIncremental static_incr;
    static_incr.solve(instance, 0);

    return EXIT_SUCCESS;
}
