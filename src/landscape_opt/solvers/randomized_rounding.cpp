#include "solvers/randomized_rounding.hpp"

static Solution job(const MutableLandscape & landscape,
                    const RestorationPlan<MutableLandscape> & plan,
                    const double B, const Solution & relaxed_solution,
                    int nb_draws) {
    Solution best_solution(landscape, plan);
    const auto nodeOptions = plan.computeNodeOptionsMap();
    const auto arcOptions = plan.computeArcOptionsMap();

    RandomChooser<RestorationPlan<MutableLandscape>::Option> option_chooser;
    for(const RestorationPlan<MutableLandscape>::Option i : plan.options()) {
        const double coef = relaxed_solution.getCoef(i);
        if(coef == 0) continue;
        option_chooser.add(i, coef);
    }

    std::vector<RestorationPlan<MutableLandscape>::Option> purschaised_options;
    double purschaised;
    DecoredLandscape<MutableLandscape> decored_landscape(landscape);
    double best_eca = 0.0;

    for(int i = 0; i < nb_draws; i++) {
        option_chooser.reset();
        purschaised_options.clear();
        purschaised = 0.0;
        decored_landscape.reset();
        while(option_chooser.canPick()) {
            RestorationPlan<MutableLandscape>::Option option =
                option_chooser.pick();
            if(purschaised + plan.getCost(option) > B) continue;
            purschaised_options.push_back(option);
            purschaised += plan.getCost(option);

            decored_landscape.apply(nodeOptions[option], arcOptions[option]);
        }
        double eca = ECA().eval(decored_landscape);

        if(eca > best_eca) {
            for(const RestorationPlan<MutableLandscape>::Option i :
                plan.options())
                best_solution.set(i, 0);
            for(RestorationPlan<MutableLandscape>::Option option :
                purschaised_options)
                best_solution.set(option, 1);

            best_eca = eca;
        }
    }

    best_solution.obj = best_eca;

    return best_solution;
}

Solution Solvers::Randomized_Rounding_ECA::solve(
    const MutableLandscape & landscape,
    const RestorationPlan<MutableLandscape> & plan, const double B) const {
    Solution solution(landscape, plan);
    const int log_level = params.at("log")->getInt();
    const int nb_draws = params.at("draws")->getInt();
    const bool parallel = params.at("parallel")->getBool();
    Chrono chrono;

    Solvers::PL_ECA_3 pl_eca_3;
    pl_eca_3.setLogLevel(log_level).setRelaxed(1);

    Solution relaxed_solution = pl_eca_3.solve(landscape, plan, B);

    // //for debug
    // std::cout << std::endl;
    // for(auto option_pair : relaxed_solution.getOptionCoefs()) {
    //     RestorationPlan<MutableLandscape>::Option* option =
    //     option_pair.first; const double coef = option_pair.second; std::cout
    //     << coef << " ";
    // }
    // std::cout << std::endl;

    if(parallel) {
        const int nb_threads = std::thread::hardware_concurrency();
        const int nb_draws_per_thread =
            nb_draws / nb_threads + (nb_draws % nb_threads > 0 ? 1 : 0);
        std::vector<Solution> v(nb_threads, Solution(landscape, plan));
        std::generate(std::execution::par, v.begin(), v.end(), [&]() {
            return job(landscape, plan, B, relaxed_solution,
                       nb_draws_per_thread);
        });
        solution = *std::max_element(v.begin(), v.end(),
                                     [](const Solution s1, const Solution s2) {
                                         return s1.obj > s2.obj;
                                     });
    } else {
        solution = job(landscape, plan, B, relaxed_solution, nb_draws);
    }

    solution.setComputeTimeMs(chrono.timeMs());

    return solution;
}