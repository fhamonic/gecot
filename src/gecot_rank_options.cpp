#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

// #include "gecot/rankers/mip_xue.hpp"
// #include "gecot/rankers/mip_pc_num.hpp"
// #include "gecot/rankers/mip_pc_num_preprocessed.hpp"
// #include "gecot/rankers/randomized_rounding.hpp"

#include "instance.hpp"
#include "parse_instance.hpp"
#include "trivial_reformulate.hpp"

#include "ranker_interfaces/abstract_ranker.hpp"
#include "ranker_interfaces/greedy_decremental_interface.hpp"
#include "ranker_interfaces/greedy_incremental_interface.hpp"
#include "ranker_interfaces/static_decremental_interface.hpp"
#include "ranker_interfaces/static_incremental_interface.hpp"

using namespace fhamonic;

void print_paragraph(std::ostream & os, std::size_t offset,
                     std::size_t column_width, const std::string & str) {
    std::size_t line_start = 0;
    while(str.size() - line_start > column_width) {
        std::size_t n = str.rfind(' ', line_start + column_width);
        if(n <= line_start) {
            os << str.substr(line_start, column_width - 1) << '-' << '\n'
               << std::string(offset, ' ');
            line_start += column_width - 1;
        } else {
            os << str.substr(line_start, n - line_start) << '\n'
               << std::string(offset, ' ');
            line_start = n + 1;
        }
    }
    os << str.substr(line_start) << '\n';
}

static bool process_command_line(
    const std::vector<std::string> & args,
    std::shared_ptr<AbstractRanker> & ranker,
    std::filesystem::path & instances_description_json_file,
    std::optional<std::filesystem::path> & opt_output_csv_file) {
    std::vector<std::shared_ptr<AbstractRanker>> ranker_interfaces{
        std::make_unique<StaticIncrementalInterface>(),
        std::make_unique<StaticDecrementalInterface>(),
        std::make_unique<GreedyIncrementalInterface>(),
        std::make_unique<GreedyDecrementalInterface>()};

    auto print_soft_name = []() {
        std::cout << "GECOT — Graph-based Ecological Connectivity "
                     "Optimization Tool\nVersion: "
                  << PROJECT_VERSION << " (built on " << __DATE__ << ")\n\n";
    };
    auto print_usage = []() {
        std::cout << R"(Usage:
  gecot_rank_options --help
  gecot_rank_options --list-algorithms
  gecot_rank_options <algorithm> --list-params
  gecot_rank_options <algorithm> <instance> [<params> ...]

)";
    };

    auto print_available_algorithms = [&ranker_interfaces]() {
        std::cout << "Available algorithms:\n";
        const std::size_t algorithm_name_max_length = std::ranges::max(
            std::ranges::views::transform(ranker_interfaces, [&](auto && s) {
                return s->name().size();
            }));
        const std::size_t offset = algorithm_name_max_length + 8;

        for(auto & s : ranker_interfaces) {
            std::cout << "  " << s->name()
                      << std::string(offset - s->name().size() - 2, ' ');
            print_paragraph(std::cout, offset, 80 - offset, s->description());
        }
        std::cout << std::endl;
        return false;
    };

    // path istead of string to silence a warning...
    std::filesystem::path output_csv_file;

    try {
        po::options_description desc("Allowed options");
        desc.add_options()("help,h", "Display this help message")(
            "list-algorithms,A", "List the available algorithms")(
            "list-params,P", "List the parameters of the chosen algorithm")(
            "algorithm,a", po::value<std::string>()->required(),
            "Algorithm to use")(
            "instance,i",
            po::value<std::filesystem::path>(&instances_description_json_file)
                ->required(),
            "Instance JSON file")(
            "output-csv,o", po::value<std::filesystem::path>(&output_csv_file),
            "Output ranks in CSV file");

        po::positional_options_description p;
        p.add("algorithm", 1);
        p.add("instance", 1);
        po::variables_map vm;
        po::parsed_options parsed = po::basic_command_line_parser(args)
                                        .options(desc)
                                        .positional(p)
                                        .allow_unregistered()
                                        .run();
        po::store(parsed, vm);

        if(vm.count("help") || args.empty()) {
            print_soft_name();
            print_usage();
            std::cout << desc << std::endl;
            return false;
        }

        if(vm.count("list-algorithms")) {
            print_soft_name();
            print_available_algorithms();
            return false;
        }

        if(vm.count("algorithm")) {
            auto ranker_name = vm["algorithm"].as<std::string>();
            bool found = false;
            for(auto & s : ranker_interfaces) {
                if(s->name() == ranker_name) {
                    ranker = std::move(s);
                    found = true;
                    break;
                }
            }
            if(!found) {
                std::cout << "Error: the argument ('" << ranker_name
                          << "') for option '--algorithm' is invalid\n\n";
                print_available_algorithms();
                return false;
            }
        }

        if(vm.count("list-params")) {
            if(!vm.count("algorithm")) {
                throw std::logic_error(std::string(
                    "Option '--list-params' requires option '--algorithm'."));
            }
            print_soft_name();
            std::cout << ranker->options_description();
            std::cout << std::endl;
            return false;
        }

        po::notify(vm);
        if(vm.count("output-csv")) {
            opt_output_csv_file.emplace(std::move(output_csv_file));
        }
        std::vector<std::string> opts =
            po::collect_unrecognized(parsed.options, po::exclude_positional);
        ranker->parse(opts);
    } catch(std::exception & e) {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    return true;
}

int main(int argc, const char * argv[]) {
#ifdef WIN32
    std::system("chcp 65001 > NUL");
#endif
    std::cout.precision(20);

    std::shared_ptr<AbstractRanker> ranker;
    std::filesystem::path instances_description_json;
    std::optional<std::filesystem::path> opt_output_csv_file;

    std::vector<std::string> args(argv + 1, argv + argc);
    bool valid_command = process_command_line(
        args, ranker, instances_description_json, opt_output_csv_file);
    if(!valid_command) return EXIT_FAILURE;

    Instance raw_instance = parse_instance(instances_description_json);
    Instance instance = trivial_reformulate_instance(raw_instance);

    gecot::instance_options_rank_t<Instance> option_ranks =
        ranker->rank_options(instance);

    std::cout << "option_ranks:" << std::endl;
    const std::size_t option_name_max_length = std::ranges::max(
        std::ranges::views::transform(instance.options(), [&](auto && o) {
            return instance.option_name(o).size();
        }));
    for(auto && o : instance.options()) {
        std::cout << "  " << instance.option_name(o)
                  << std::string(option_name_max_length + 1 -
                                     instance.option_name(o).size(),
                                 ' ')
                  << option_ranks[o] << '\n';
    }
    std::cout << std::endl;

    if(opt_output_csv_file.has_value()) {
        std::ofstream output_file(opt_output_csv_file.value());
        if(output_file.is_open()) {
            output_file << "option_id,potential\n";
            for(auto && o : instance.options()) {
                output_file << instance.option_name(o) << ',' << option_ranks[o]
                            << '\n';
            }
            output_file << std::endl;
        } else
            std::cerr << "ERR: '" << opt_output_csv_file.value()
                      << "' not opened" << std::endl;
    }

    return EXIT_SUCCESS;
}
