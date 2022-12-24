#ifndef G6W1_HPP
#define G6W1_HPP

#include <fstream>
#include <chrono>
#include <memory>
#include <map>

#include <ros/package.h>
#include "ros/ros.h"

#include "libbsn/processor/Processor.hpp"
#include "libbsn/utils/utils.hpp"

/*#include "component/CentralHub.hpp"*/
#include "component/CentralHubWeather.hpp" /*centralhub_weather (?)*/

#include "archlib/target_system/Component.hpp"
#include "archlib/AdaptationCommand.h"

#include "messages/SensorData.h"
#include "messages/TargetSystemData.h"

class G6W1 : public CentralHub_Weather {
    
    public:
        G6W1(int &argc, char **argv, const std::string &name);
        virtual ~G6W1();

    private:
        G6W1(const G6W1 & /*obj*/);
        G6W1 &operator=(const G6W1 & /*obj*/);

        std::string makePacket();
        std::vector<std::string> getWeatherStatus();
        int32_t getSensorId(std::string type);

    public:
        virtual void setUp();
        virtual void tearDown();   

        virtual void collect(const messages::SensorData::ConstPtr& sensor_data);
        virtual void process();
        virtual void transfer();

    private:
        double weather_status;

        double env_ther_risk;
        double baro_risk;
        double hygro_risk;
       
        double env_ther_batt;
        double baro_batt;
        double hygro_batt;

        double env_ther_raw;
        double baro_raw;
        double hygro_raw;

        ros::Publisher pub;
        bool lost_packt;
};

#endif 