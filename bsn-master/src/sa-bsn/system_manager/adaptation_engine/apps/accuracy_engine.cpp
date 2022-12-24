#include "engine/AccuracyEngine.hpp"

#include "ros/ros.h"

int main(int argc, char **argv) {
    AccuracyEngine engine(argc, argv, "engine");
    return engine.run();
}
