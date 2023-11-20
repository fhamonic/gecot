#ifndef GECOT_STATIC_INCREMENTAL_INTERFACE_HPP
#define GECOT_STATIC_INCREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "gecot/rankers/static_incremental.hpp"

#include "instance.hpp"
#include "ranker_interfaces/abstract_ranker.hpp"

namespace fhamonic {

class StaticIncrementalInterface : public AbstractRanker {
private:
    gecot::rankers::StaticIncremental ranker;
    boost::program_options::options_description desc;

public:
    StaticIncrementalInterface() : desc(name() + " options") {
        desc.add_options()("verbose,v", "Log the algorithm steps")(
            "parallel,p", "Use multithreaded version");
    }

    void parse(const std::vector<std::string> & args) {
        boost::program_options::variables_map vm;
        boost::program_options::store(
            boost::program_options::command_line_parser(args)
                .options(desc)
                .run(),
            vm);
        po::notify(vm);

        ranker.verbose = vm.count("verbose") > 0;
        ranker.parallel = vm.count("parallel") > 0;
    }

    typename gecot::instance_options_rank_t<Instance> rank_options(
        const Instance & instance) const {
        return ranker.rank_options(instance);
    };

    std::string name() const { return "static_incremental"; }
    std::string description() const {
        return "From the base landscape, add the options with the best "
               "gain/cost ratio.";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "static_incremental"; }
};

}  // namespace fhamonic

#endif  // GECOT_STATIC_INCREMENTAL_INTERFACE_HPP