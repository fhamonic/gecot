#include "solvers/glutton_eca_dec.hpp"

Solution Solvers::Glutton_ECA_Dec::solve(
    const MutableLandscape & landscape,
    const RestorationPlan<MutableLandscape> & plan, const double B) const {
    using Option = RestorationPlan<MutableLandscape>::Option;
    Solution solution(landscape, plan);
    const int log_level = params.at("log")->getInt();
    const bool parallel = params.at("parallel")->getBool();
    Chrono chrono;

    const MutableLandscape::Graph & graph = landscape.getNetwork();
    const auto nodeOptions = plan.computeNodeOptionsMap();
    const auto arcOptions = plan.computeArcOptionsMap();

    std::vector<Option> options;
    std::vector<Option> free_options;

    double purchaised = 0.0;
    for(const Option i : plan.options()) {
        purchaised += plan.getCost(i);
        solution.add(i);
        options.push_back(i);
    }

    double prec_eca =
        ECA().eval(Helper::decore_landscape(landscape, plan, solution));
    if(log_level > 1) {
        std::cout << "base purchaised: " << purchaised << std::endl;
        std::cout << "base ECA: " << prec_eca << std::endl;
    }

    auto min_option = [](std::pair<double, Option> p1,
                         std::pair<double, Option> p2) {
        return (p1.first < p2.first) ? p1 : p2;
    };
    auto compute_min_option = [&landscape, &plan, &nodeOptions, &arcOptions,
                               &prec_eca, &solution](Option option) {
        DecoredLandscape<MutableLandscape> decored_landscape(landscape);

        for(const Option i : plan.options()) {
            if(i == option) continue;
            decored_landscape.apply(nodeOptions[i], arcOptions[i], solution[i]);
        }
        const double eca = ECA().eval(decored_landscape);
        const double ratio = (prec_eca - eca) / plan.getCost(option);

        return std::pair<double, Option>(ratio, option);
    };
    while(purchaised > B) {
        std::pair<double, Option> worst =
            parallel
                ? std::transform_reduce(
                      std::execution::par, options.begin(), options.end(),
                      std::make_pair(std::numeric_limits<double>::max(), -1),
                      min_option, compute_min_option)
                : std::transform_reduce(
                      std::execution::seq, options.begin(), options.end(),
                      std::make_pair(std::numeric_limits<double>::max(), -1),
                      min_option, compute_min_option);

        const double worst_ratio = worst.first;
        Option worst_option = worst.second;

        if(worst_option == -1) break;

        options.erase(std::find(options.begin(), options.end(), worst_option));
        free_options.push_back(worst_option);

        const double worst_option_cost = plan.getCost(worst_option);
        solution.remove(worst_option);
        purchaised -= worst_option_cost;
        prec_eca -= worst_ratio * worst_option_cost;

        if(log_level > 1) {
            std::cout << "remove option: " << worst_option << " costing "
                      << worst_option_cost << std::endl;
            if(log_level > 2) {
                for(auto const & [u, quality_gain] : nodeOptions[worst_option])
                    std::cout << "\tn " << graph.id(u) << std::endl;
                for(auto const & [a, restored_probability] :
                    arcOptions[worst_option]) {
                    MutableLandscape::Node source = graph.source(a);
                    MutableLandscape::Node target = graph.target(a);
                    std::cout << "\ta "
                              << " " << graph.id(source) << " "
                              << graph.id(target) << std::endl;
                }
            }
            std::cout << "current purchaised: " << purchaised << std::endl;
            std::cout << "current ECA: " << prec_eca << std::endl;
            std::cout << "remaining : " << options.size() << std::endl;
        }
    }

    auto max_option = [](std::pair<double, Option> p1,
                         std::pair<double, Option> p2) {
        return (p1.first > p2.first) ? p1 : p2;
    };
    auto compute_max_option = [&landscape, &plan, &nodeOptions, &arcOptions,
                               &prec_eca, &solution](Option option) {
        DecoredLandscape<MutableLandscape> decored_landscape(landscape);
        for(const Option i : plan.options()) {
            decored_landscape.apply(nodeOptions[i], arcOptions[i], solution[i]);
        }
        decored_landscape.apply(nodeOptions[option], arcOptions[option]);
        const double eca = ECA().eval(decored_landscape);
        const double ratio = (eca - prec_eca) / plan.getCost(option);

        std::cout << ratio << std::endl;

        return std::make_pair(ratio, option);
    };
    for(;;) {
        // std::cout << "il reste des options: " << free_options.size() << std::endl;

        // for(auto option : free_options) {
        //     std::cout << "option: " << option
        //               << "\tcost:" << purchaised + plan.getCost(option) << "\tbudget:" << B
        //               << std::endl;
        // }

        free_options.erase(std::remove_if(
            free_options.begin(), free_options.end(),
            [&](Option i) { return purchaised + plan.getCost(i) > B; }), free_options.end());

        // std::cout << "il en reste : " << free_options.size() << std::endl;

        if(free_options.empty()) break;

        std::pair<double, Option> best =
            parallel ? std::transform_reduce(
                           std::execution::par, free_options.begin(),
                           free_options.end(), std::make_pair(0.0, -1),
                           max_option, compute_max_option)
                     : std::transform_reduce(
                           std::execution::seq, free_options.begin(),
                           free_options.end(), std::make_pair(0.0, -1),
                           max_option, compute_max_option);

        const double best_ratio = best.first;
        Option best_option = best.second;

        std::cout << "best option : " << best_option << std::endl;

        if(best_option == -1) break;

        free_options.erase(
            std::find(free_options.begin(), free_options.end(), best_option));

        const double best_option_cost = plan.getCost(best_option);
        assert(purchaised + best_option_cost <= B);
        solution.add(best_option);
        purchaised += best_option_cost;
        prec_eca += best_ratio * best_option_cost;

        if(log_level > 1) {
            std::cout << "add option: " << best_option_cost << std::endl;
            if(log_level > 2) {
                for(auto const & [u, quality_gain] : nodeOptions[best_option])
                    std::cout << "\tn " << graph.id(u) << std::endl;
                for(auto const & [a, restored_probability] :
                    arcOptions[best_option]) {
                    MutableLandscape::Node source = graph.source(a);
                    MutableLandscape::Node target = graph.target(a);
                    std::cout << "\ta "
                              << " " << graph.id(source) << " "
                              << graph.id(target) << std::endl;
                }
            }
            std::cout << "current purchaised: " << purchaised << std::endl;
            std::cout << "current ECA: " << prec_eca << std::endl;
            std::cout << "remaining : " << free_options.size() << std::endl;
        }
    }

    solution.setComputeTimeMs(chrono.timeMs());
    solution.obj = prec_eca;
    if(log_level >= 1) {
        std::cout << name()
                  << ": Complete solving : " << solution.getComputeTimeMs()
                  << " ms" << std::endl;
        std::cout << name() << ": ECA from obj : " << solution.obj << std::endl;
    }

    return solution;
}