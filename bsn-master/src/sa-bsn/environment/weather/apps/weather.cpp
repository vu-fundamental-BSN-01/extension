#include "WeatherModule.hpp"

int main(int argc, char **argv) {
    WeatherModule weather(argc, argv, "weather");
    return weather.run();
}

