#include "component/g5w1_2/G5W1_2.hpp"

int32_t main(int32_t argc, char **argv) {
    G5W1_2 g5w1_2(argc, argv, "barometer");
    return g5w1_2.run();
}
