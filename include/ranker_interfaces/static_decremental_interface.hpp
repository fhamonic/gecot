#ifndef LSCP_STATIC_DECREMENTAL_INTERFACE_HPP
#define LSCP_STATIC_DECREMENTAL_INTERFACE_HPP

#include <sstream>

#include <boost/program_options.hpp>

#include "landscape_opt/rankers/static_decremental.hpp"

#include "instance.hpp"
#include "ranker_interfaces/abstract_ranker.hpp"

namespace fhamonic {

class StaticDecrementalInterface : public AbstractRanker {
private:
    landscape_opt::rankers::StaticDecremental ranker;
    boost::program_options::options_description desc;

public:
    StaticDecrementalInterface() : desc(name() + " options") {
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

    typename landscape_opt::instance_options_rank_t<Instance> rank_options(
        const Instance & instance) const {
        return ranker.rank_options(instance);
    };

    std::string name() const { return "static_decremental"; }
    std::string description() const {
        return "From the improved landscape, remove the options with the worst "
               "gain/cost ratio.";
    }
    std::string options_description() const {
        std::ostringstream s;
        s << desc;
        return s.str();
    }
    std::string string() const { return "static_decremental"; }
};

}  // namespace fhamonic

#endif  // LSCP_STATIC_DECREMENTAL_INTERFACE_HPP