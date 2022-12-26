#include <ros/ros.h>
#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/utils/utils.hpp"
#include "libbsn/range/Range.hpp"
#include <string>
#include "services/WeatherData.h"
#include <ros/console.h>

#include "archlib/ROSComponent.hpp"

class WeatherModule : public arch::ROSComponent {
    public:
        WeatherModule(int &argc, char **argv, std::string name);
        ~WeatherModule();

        void setUp();
        void tearDown();
        void body();

    private:
        bool getWeatherData(services::WeatherData::Request &request, services::WeatherData::Response &response);
        bsn::generator::DataGenerator configureDataGenerator(const std::string& weatherFactors);

        std::map<std::string, bsn::generator::DataGenerator> weatherData;
        std::map<std::string, double> weatherFactorsFrequencies;
        std::map<std::string, double> weatherFactorsChanges;
        std::map<std::string, double> weatherFactorsOffsets;

        double frequency;
        double period;
        ros::NodeHandle nh;
        ros::ServiceServer service;
};