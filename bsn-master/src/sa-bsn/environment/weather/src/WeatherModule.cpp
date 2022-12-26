#include <WeatherModule.hpp>

WeatherModule::WeatherModule(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

WeatherModule::~WeatherModule() {}

void WeatherModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static
    std::string weatherFactors;
    service = nh.advertiseService("getWeatherData", &WeatherModule::getWeatherData, this);
    double aux;

    frequency = 1000;

    // Get what vital signs this module will simulate
    nh.getParam("weatherFactors", weatherFactors);

    // Removes white spaces from weatherFactors
    weatherFactors.erase(std::remove(weatherFactors.begin(), weatherFactors.end(),' '), weatherFactors.end());

    std::vector<std::string> splittedWeatherFactors = bsn::utils::split(weatherFactors, ',');

    for (std::string s : splittedWeatherFactors) {
        weatherFactorsFrequencies[s] = 0;
        nh.getParam(s + "_Change", aux);
        weatherFactorsChanges[s] = 1/aux;
        nh.getParam(s + "_Offset", weatherFactorsOffsets[s]);
    }

    for (const std::string& s : splittedWeatherFactors) {
        weatherData[s] = configureDataGenerator(s);
    }

    rosComponentDescriptor.setFreq(frequency);
    
    period = 1/frequency;
}

bsn::generator::DataGenerator WeatherModule::configureDataGenerator(const std::string& weatherFactors) {
    srand(time(NULL));
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    // std::cout << vitalSign << std::endl;
    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(weatherFactors + "_State" + std::to_string(j), s);
            t_probs = bsn::utils::split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "_LowRisk", s);
    lrs = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk0", s);
    mrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk0", s);
    hrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk1", s);
    mrs1 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk1", s);
    hrs1 = bsn::utils::split(s, ',');

    ranges[0] = bsn::range::Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
    ranges[1] = bsn::range::Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
    ranges[2] = bsn::range::Range(std::stod(lrs[0]), std::stod(lrs[1]));
    ranges[3] = bsn::range::Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
    ranges[4] = bsn::range::Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

    bsn::generator::Markov markov(transitions, ranges, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

    return dataGenerator;
}

void WeatherModule::tearDown() {}

bool WeatherModule::getWeatherData(services::WeatherData::Request &request, 
                                services::WeatherData::Response &response) {
    
    response.data = weatherData[request.weatherFactors].getValue();
    
    std::cout << "Send " + request.weatherFactors + " data." << std::endl;

    return true;
}

void WeatherModule::body() {
    for (auto &p : weatherFactorsFrequencies) {
        
        if (p.second >= (weatherFactorsChanges[p.first] + weatherFactorsOffsets[p.first])) {
            weatherData[p.first].nextState();
            p.second = weatherFactorsOffsets[p.first];
            std::cout << "Changed " + p.first + " state." << std::endl;
        } else {
            p.second += period;
        }
        
    }
}

