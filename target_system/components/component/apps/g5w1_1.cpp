#include "component/g5w1_1/G5W1_1.hpp"

int32_t main(int32_t argc, char **argv) {
    G5W1_1 g5w1_1(argc, argv, "thermometer");
    return g5w1_1.run();
}
