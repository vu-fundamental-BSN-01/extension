#include "component/g6w1/G6W1.hpp"

int32_t main(int32_t argc, char **argv) {
    G6W1 g6w1(argc, argv, "centralhubweather");
    return g6w1.run();
}

