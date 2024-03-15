#ifndef RANDOM_CHOOSER_HPP
#define RANDOM_CHOOSER_HPP

#include <random>
#include <vector>

#include <cassert>

template <typename T>
class RandomChooser {
private:
    std::default_random_engine gen;
    double distrib_sum;
    std::vector<std::pair<T, double>> elements;
    double rest;
    std::size_t picked;

public:
    RandomChooser() : distrib_sum{0.0}, rest{0.0}, picked{0} {};
    RandomChooser(int seed) : distrib_sum{0.0}, rest{0.0}, picked{0} {
        gen.seed(seed);
    };
    ~RandomChooser(){};

    void add(T e, double distrib);

    bool canPick();
    T pick();
    void reset();
};

template <typename T>
void RandomChooser<T>::add(T e, double distrib) {
    elements.emplace_back(e, distrib);
    distrib_sum += distrib;
    reset();
}

template <typename T>
bool RandomChooser<T>::canPick() {
    return picked < elements.size();
}

template <typename T>
T RandomChooser<T>::pick() {
    assert(canPick());
    std::uniform_real_distribution<> dis(0, rest);
    double rv = dis(gen);

    int i = 0;
    for(;; i++) {
        rv -= elements[i].second;
        if(rv < 0) break;
    }
    std::pair tmp = elements[i];
    rest -= tmp.second;
    picked++;

    elements[i] = elements[elements.size() - picked];
    elements[elements.size() - picked] = tmp;

    return tmp.first;
}

template <typename T>
void RandomChooser<T>::reset() {
    rest = distrib_sum;
    picked = 0;
}

#endif