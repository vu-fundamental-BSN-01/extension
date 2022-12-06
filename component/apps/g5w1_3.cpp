#include "component/g5w1_3/G5W1_3.hpp"

int32_t main(int32_t argc, char **argv) {
    G5W1_3 g5w1_3(argc, argv, "hygrometer");
    return g5w1_3.run();
}
