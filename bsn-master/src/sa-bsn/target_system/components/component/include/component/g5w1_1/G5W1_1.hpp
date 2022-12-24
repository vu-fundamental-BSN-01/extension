#ifndef G5W1_1_HPP
#define G5W1_1_HPP

#include <string>
#include <exception>

#include "ros/ros.h"

#include "libbsn/range/Range.hpp"
#include "libbsn/resource/Battery.hpp"
#include "libbsn/generator/Markov.hpp"
#include "libbsn/generator/DataGenerator.hpp"
#include "libbsn/filters/MovingAverage.hpp"
#include "libbsn/utils/utils.hpp"
#include "libbsn/configuration/SensorConfiguration.hpp"

#include "component/Sensor.hpp"

#include "messages/SensorData.h"  
#include "services/WeatherData.h" /* should be weather data (?) */

class G5W1_1 : public Sensor {
    	
  	public:
    	G5W1_1(int &argc, char **argv, const std::string &name);
    	~G5W1_1();

	private:
      	G5W1_1(const G5W1_1 &);
    	G5W1_1 &operator=(const G5W1_1 &);

		std::string label(double &risk);
    
	public:
		virtual void setUp();
    	virtual void tearDown();

        double collect();
        double process(const double &data);
        void transfer(const double &data);

  	private:
		bsn::generator::Markov markov;
		bsn::generator::DataGenerator dataGenerator;		
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		ros::NodeHandle handle;
		ros::Publisher data_pub;
		ros::ServiceClient client;	

		double collected_risk;

};

#endif 
