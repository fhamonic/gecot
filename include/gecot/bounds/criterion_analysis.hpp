#ifndef GECOT_BOUNDS_CRITERION_ANALYSIS_HPP
#define GECOT_BOUNDS_CRITERION_ANALYSIS_HPP

#include <algorithm>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <variant>
#include <vector>

#include <fmt/core.h>

#include "gecot/concepts/instance.hpp"
#include "gecot/criterion.hpp"

namespace gecot {

// φ(z) = constant + Σ_k weights[k] z_k
struct criterion_linear_form {
    double constant = 0.0;
    std::vector<double> weights;
};

namespace detail {

struct linear_form_visitor {
    std::size_t num_cases;

    using result = std::optional<criterion_linear_form>;

    result operator()(const criterion_constant & c) const {
        return criterion_linear_form{c, std::vector<double>(num_cases, 0.0)};
    }
    result operator()(const criterion_var & v) const {
        criterion_linear_form form{0.0, std::vector<double>(num_cases, 0.0)};
        form.weights.at(v) = 1.0;
        return form;
    }
    result operator()(const criterion_sum & f) const {
        criterion_linear_form form{0.0, std::vector<double>(num_cases, 0.0)};
        for(auto && e : f.values) {
            const result term = std::visit(*this, e);
            if(!term) return std::nullopt;
            form.constant += term->constant;
            for(std::size_t k = 0; k < num_cases; ++k)
                form.weights[k] += term->weights[k];
        }
        return form;
    }
    // linear if at most one factor depends on the case scores: the scaled
    // criterion of trivial_reformulate_instance puts its weight first
    result operator()(const criterion_product & f) const {
        criterion_linear_form form{1.0, std::vector<double>(num_cases, 0.0)};
        for(auto && e : f.values) {
            result factor = std::visit(*this, e);
            if(!factor) return std::nullopt;
            if(is_constant(*factor)) {
                form.constant *= factor->constant;
                for(auto & w : form.weights) w *= factor->constant;
            } else if(is_constant(form)) {
                const double c = form.constant;
                form = std::move(*factor);
                form.constant *= c;
                for(auto & w : form.weights) w *= c;
            } else {
                return std::nullopt;
            }
        }
        return form;
    }
    result operator()(const criterion_min & f) const {
        if(f.values.size() != 1) return std::nullopt;
        return std::visit(*this, f.values.front());
    }

    static bool is_constant(const criterion_linear_form & form) {
        return std::ranges::all_of(form.weights,
                                   [](double w) { return w == 0.0; });
    }
};

// over non-negative case scores
struct monotony_visitor {
    struct result {
        bool non_decreasing;
        bool non_negative;
        std::optional<double> constant;  // if no case score appears
    };

    result operator()(const criterion_constant & c) const {
        return {true, c >= 0, c};
    }
    result operator()(const criterion_var &) const {
        return {true, true, std::nullopt};
    }
    result operator()(const criterion_sum & f) const {
        result r{true, true, 0.0};
        for(auto && e : f.values) {
            const result term = std::visit(*this, e);
            r.non_decreasing &= term.non_decreasing;
            r.non_negative &= term.non_negative;
            r.constant = r.constant && term.constant
                             ? std::optional(*r.constant + *term.constant)
                             : std::nullopt;
        }
        return r;
    }
    result operator()(const criterion_min & f) const {
        result r{true, true, std::nullopt};
        bool all_constant = true;
        double min = std::numeric_limits<double>::max();
        for(auto && e : f.values) {
            const result term = std::visit(*this, e);
            r.non_decreasing &= term.non_decreasing;
            r.non_negative &= term.non_negative;
            all_constant &= term.constant.has_value();
            if(term.constant) min = std::min(min, *term.constant);
        }
        if(all_constant) r.constant = min;
        return r;
    }
    // a positive constant times one non-decreasing factor, or times
    // non-negative non-decreasing factors, is non-decreasing
    result operator()(const criterion_product & f) const {
        double constant = 1.0;
        std::size_t num_factors = 0;
        bool factors_non_decreasing = true, factors_non_negative = true;
        for(auto && e : f.values) {
            const result factor = std::visit(*this, e);
            if(factor.constant) {
                constant *= *factor.constant;
                continue;
            }
            ++num_factors;
            factors_non_decreasing &= factor.non_decreasing;
            factors_non_negative &= factor.non_negative;
        }
        if(num_factors == 0 || constant == 0) {
            const double value = num_factors == 0 ? constant : 0.0;
            return {true, value >= 0, value};
        }
        return {constant > 0 && factors_non_decreasing &&
                    (num_factors == 1 || factors_non_negative),
                constant > 0 && factors_non_negative, std::nullopt};
    }
};

struct cases_visitor {
    std::vector<case_id_t> & cases;

    void operator()(const criterion_constant &) const {}
    void operator()(const criterion_var & v) const { cases.push_back(v); }
    template <typename F>
    void operator()(const F & f) const {
        for(auto && e : f.values) std::visit(*this, e);
    }
};

}  // namespace detail

inline std::optional<criterion_linear_form> linear_form(
    const criterion_formula & criterion, std::size_t num_cases) {
    return std::visit(detail::linear_form_visitor{num_cases}, criterion);
}

// in every case score, over non-negative case scores; a sufficient test
inline bool is_non_decreasing(const criterion_formula & criterion) {
    return std::visit(detail::monotony_visitor{}, criterion).non_decreasing;
}

// A sufficient test that the criterion of instance never decreases when a
// case score increases: it is linear with non-negative weights once its terms
// are added up, as 2 z - z, whose second term is_non_decreasing refuses, or
// is_non_decreasing accepts it.
template <instance_c I>
bool has_non_decreasing_criterion(const I & instance) {
    const auto form =
        linear_form(instance.criterion(), std::ranges::size(instance.cases()));
    return (form &&
            std::ranges::all_of(form->weights,
                                [](const double w) { return w >= 0.0; })) ||
           is_non_decreasing(instance.criterion());
}

inline std::vector<case_id_t> criterion_cases(
    const criterion_formula & criterion) {
    std::vector<case_id_t> cases;
    std::visit(detail::cases_visitor{cases}, criterion);
    std::ranges::sort(cases);
    const auto [first, last] = std::ranges::unique(cases);
    cases.erase(first, last);
    return cases;
}

// A bound on the criterion from a bound on its weighted case scores:
// weighted_bound(w), for weights w >= 0 indexed by case id, must bound
// Σ_k w_k pc_k(X) over the solutions X within the budget. A linear criterion
// with non-negative weights takes one call, a non-decreasing one a call per
// case it reads.
template <instance_c I, typename F>
double bound_criterion(const I & instance, std::string_view method,
                       F && weighted_bound) {
    const auto & criterion = instance.criterion();
    const std::size_t num_cases = std::ranges::size(instance.cases());
    if(const auto form = linear_form(criterion, num_cases);
       form &&
       std::ranges::all_of(form->weights, [](double w) { return w >= 0.0; })) {
        return form->constant +
               weighted_bound(std::span<const double>(form->weights));
    }
    if(!is_non_decreasing(criterion))
        throw std::invalid_argument(
            fmt::format("{}: the criterion is not non-decreasing in the case "
                        "scores",
                        method));
    auto case_bounds = instance.template create_case_map<double>();
    std::vector<double> unit_weights(num_cases, 0.0);
    for(auto && instance_case : instance.cases())
        case_bounds[instance_case.id()] = 0.0;
    for(const case_id_t k : criterion_cases(criterion)) {
        unit_weights[k] = 1.0;
        case_bounds[k] = weighted_bound(std::span<const double>(unit_weights));
        unit_weights[k] = 0.0;
    }
    return instance.eval_criterion(case_bounds);
}

}  // namespace gecot

#endif  // GECOT_BOUNDS_CRITERION_ANALYSIS_HPP
