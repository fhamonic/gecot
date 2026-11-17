#pragma once

#include <algorithm>
#include <cstddef>
#include <ranges>
#include <string_view>

#include <spdlog/spdlog.h>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/column_manager.hpp"

#include "gecot/concepts/instance.hpp"

namespace gecot {
namespace solvers {

// What path_lp and tree_lp share of their column generation
// (docs/formulations/relaxations.md).

// The columns of a master: Key names a column, its value is the data; each
// has a reduced cost, and in the master an age.
template <typename LP, typename Key>
using column_manager = mippp::colgen::column_manager<
    LP, Key, mippp::colgen::property_list<mippp::colgen::reduced_cost>,
    mippp::colgen::property_list<mippp::colgen::reduced_cost,
                                 mippp::colgen::age>,
    double>;
double reduced_cost_of(const auto & entry) {
    return entry.state.template get<mippp::colgen::reduced_cost>();
}

// Logs at -vv the value of a master and the x_value(i) of each option.
template <instance_c I>
void log_master(const I & instance, const std::string_view name,
                const double value, auto && x_value) {
    if(!spdlog::should_log(spdlog::level::trace)) return;
    const auto option_name_max_length = std::ranges::max(std::views::transform(
        instance.options(),
        [&](auto && i) { return instance.option_name(i).size(); }));
    spdlog::trace("{} relaxation has value: {}", name, value);
    for(auto && i : instance.options())
        spdlog::trace("    {:<{}} {}", instance.option_name(i),
                      option_name_max_length, x_value(i));
}

// The pricing test, at the master's dual tolerance: a column enters if its
// reduced cost is above this margin, sigma being the dual of the uniqueness
// row of its pair or target.
inline double pricing_margin(const double sigma) { return 1e-7 * sigma + 1e-6; }

// The columns in the master for more than evict_age rounds and unattractive,
// so nonbasic. Warns that --evict-age is ignored on a solver that cannot
// remove columns.
template <typename LP>
auto unattractive_columns(const std::size_t evict_age) {
    namespace colgen = mippp::colgen;
    if constexpr(!mippp::has_remove_variable<LP>)
        if(evict_age)
            spdlog::warn(
                "This solver cannot remove columns: --evict-age ignored");
    return colgen::conjunction<colgen::above<colgen::age>,
                               colgen::negative<colgen::reduced_cost>>{
        colgen::above<colgen::age>{evict_age},
        colgen::negative<colgen::reduced_cost>{1e-6}};
}

}  // namespace solvers
}  // namespace gecot
