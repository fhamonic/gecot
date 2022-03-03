#ifndef BPO_UTILS_HPP
#define BPO_UTILS_HPP

#include <boost/program_options.hpp>

namespace BPOUtils {
void conflicting_options(const boost::program_options::variables_map & vm,
                         const std::string & opt1, const std::string & opt2) {
    if(vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) &&
       !vm[opt2].defaulted()) {
        throw std::logic_error(std::string("Conflicting options '") + opt1 +
                               "' and '" + opt2 + "'.");
    }
}

std::pair<std::string, std::string> split_equality_str(
    const std::string & str) {
    const int split_index = str.find("=");
    if(split_index == -1) return std::make_pair(str, std::string());
    return std::make_pair(str.substr(0, split_index),
                          str.substr(split_index + 1, -1));
}

}  // namespace BPOUtils

#endif  // BPO_UTILS