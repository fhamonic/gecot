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


    std::cout << "number of vertices " << melon::nb_vertices(instance_case.graph()) << std::endl;
    std::cout << "number of arcs " << melon::nb_arcs(instance_case.graph()) << std::endl;

    const double seq_eca = fhamonic::landscape_opt::eca(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "seq ECA = " << seq_eca << " in "
              << static_cast<double>(chrono.lap_time_us()) / 1000.0 << " ms" << std::endl;

    const double par_eca = fhamonic::landscape_opt::parallel_eca(
        instance_case.graph(), instance_case.vertex_quality_map(),
        instance_case.arc_probability_map());
    std::cout << "par ECA = " << par_eca << " in "
              << static_cast<double>(chrono.lap_time_us()) / 1000.0 << " ms" << std::endl;

    std::cout << "number of options " << instance.options().size() << std::endl;

    landscape_opt::solvers::StaticIncremental static_incr;
    auto solution = static_incr.solve(instance, 10000);

    std::cout << "found "
              << landscape_opt::compute_solution_score(instance, solution)
              << " in " << chrono.lap_time_ms() << " ms" << std::endl;

    return EXIT_SUCCESS;
}
