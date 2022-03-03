#include "solvers/glutton_eca_inc.hpp"

Solution Solvers::Glutton_ECA_Inc::solve(
    const MutableLandscape & landscape,
    const RestorationPlan<MutableLandscape> & plan, const double B) const {
    Solution solution(landscape, plan);
    const int log_level = params.at("log")->getInt();
    const bool parallel = params.at("parallel")->getBool();
    Chrono chrono;

    const MutableLandscape::Graph & graph = landscape.getNetwork();
    const auto nodeOptions = plan.computeNodeOptionsMap();
    const auto arcOptions = plan.computeArcOptionsMap();

    std::vector<RestorationPlan<MutableLandscape>::Option> options;
    double purchaised = 0.0;

    for(RestorationPlan<MutableLandscape>::Option i : plan.options())
        options.push_back(i);

    double prec_eca = ECA().eval(landscape);
    if(log_level > 1) {
        std::cout << "base ECA: " << prec_eca << std::endl;
    }

    auto max_option =
        [](std::pair<double, RestorationPlan<MutableLandscape>::Option> p1,
           std::pair<double, RestorationPlan<MutableLandscape>::Option> p2) {
            return (p1.first > p2.first) ? p1 : p2;
        };
    auto compute_option =
        [&landscape, &plan, &nodeOptions, &arcOptions, &prec_eca,
         &solution](RestorationPlan<MutableLandscape>::Option option) {
            DecoredLandscape<MutableLandscape> decored_landscape(landscape);
            for(RestorationPlan<MutableLandscape>::Option i : plan.options()) {
                decored_landscape.apply(nodeOptions[i], arcOptions[i],
                                        solution.getCoef(i));
            }
            decored_landscape.apply(nodeOptions[option], arcOptions[option]);
            const double eca = ECA().eval(decored_landscape);
            const double ratio = (eca - prec_eca) / plan.getCost(option);

            return std::pair<double, RestorationPlan<MutableLandscape>::Option>(
                ratio, option);
        };
    for(;;) {
        auto new_end_it =
            std::remove_if(options.begin(), options.end(),
                           [&](RestorationPlan<MutableLandscape>::Option i) {
                               return plan.getCost(i) > B - purchaised;
                           });
        options.erase(new_end_it, options.end());

        if(options.empty()) break;

        std::pair<double, RestorationPlan<MutableLandscape>::Option> best =
            parallel
                ? std::transform_reduce(std::execution::par, options.begin(),
                                        options.end(), std::make_pair(0.0, -1),
                                        max_option, compute_option)
                : std::transform_reduce(std::execution::seq, options.begin(),
                                        options.end(), std::make_pair(0.0, -1),
                                        max_option, compute_option);

        const double best_ratio = best.first;
        RestorationPlan<MutableLandscape>::Option best_option = best.second;

        if(best_option == -1) break;

        options.erase(std::find(options.begin(), options.end(), best_option));

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
            std::cout << "remaining : " << options.size() << std::endl;
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