#ifndef G5W1_3_HPP
#define G5W1_3_HPP

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
#include "services/PatientData.h" /* should be weather data (?) */

class G5W1_3 : public Sensor {
    	
  	public:
    	G5W1_3(int &argc, char **argv, const std::string &name);
    	~G5W1_3();

	private:
      	G5W1_3(const G5W1_3 &);
    	G5W1_3 &operator=(const G5W1_3 &);

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
