#ifndef GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP
#define GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/rankers/greedy_decremental.hpp"

#include "ranker_interfaces/abstract_ranker.hpp"

namespace fhamonic {

class GreedyDecrementalInterface : public AbstractRanker {
private:
    gecot::rankers::GreedyDecremental ranker;
    boost::program_options::options_description desc;

public:
    GreedyDecrementalInterface() : desc(name() + " options") {
        desc.add_options()("parallel,p", "Use multithreaded version");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

        ranker.parallel = vm.count("parallel") > 0;
    }

    typename gecot::instance_options_rank_t<Instance> rank_options(
        const Instance & instance) const {
        return ranker.rank_options(instance);
    };

    std::string name() const { return "greedy_decremental"; }
    std::string description() const {
        return "From the improved landscape, iteratively remove the option with "
               "the worst gain/cost ratio (Zonation Algorithm).";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "greedy_decremental"; }
};

}  // namespace fhamonic

#endif  // GECOT_GREEDY_DECREMENTAL_INTERFACE_HPP