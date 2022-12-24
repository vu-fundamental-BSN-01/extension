#include "component/g6w1/G6W1.hpp"
#define W(x) std::cerr << #x << " = " << x << std::endl;

#define BATT_UNIT 0.001

using namespace bsn::processor;

G6W1::G6W1(int &argc, char **argv, const std::string &name) : lost_packt(false),
    CentralHubWeather(argc, argv, name, true, bsn::resource::Battery("chw_batt", 100, 100, 1) ),
    weather_status(0.0) {}
	
G6W1::~G6W1() {}

std::vector<std::string> G6W1::getWeatherStatus() {
    std::string sensor_risk_str;
    std::string env_ther;
    std::string baro;
    std::string hygro;

    for (int i = 0; i < 6; i++) {
        double sensor_risk = data_buffer[i].back();


        if (sensor_risk > 0 && sensor_risk <= 20) {
            sensor_risk_str = "low risk";
        } else if (sensor_risk > 20 && sensor_risk <= 65) {
            sensor_risk_str = "moderate risk";
        } else if (sensor_risk > 65 && sensor_risk <= 100) {
            sensor_risk_str = "high risk";
        } else {
            sensor_risk_str = "unknown";
        }

        if (i == 0) {
            env_ther = sensor_risk_str;
            env_ther_risk = sensor_risk;
        } else if (i == 1) {
            baro = sensor_risk_str;
            baro_risk = sensor_risk;
        } else if (i == 2) {
            hygro = sensor_risk_str;
            hygro_risk = sensor_risk;
        }
    }

    std::vector<std::string> v = {env_ther, baro, hygro};  
    return v;
}

void G6W1::setUp() {
    Component::setUp();

    ros::NodeHandle config;

    double freq;
    config.getParam("frequency", freq);
    rosComponentDescriptor.setFreq(freq);

    for (std::vector<std::list<double>>::iterator it = data_buffer.begin();
        it != data_buffer.end(); ++it) {
            (*it) = {0.0};
    }

    pub = config.advertise<messages::TargetSystemData>("TargetSystemData", 10);
}

void G6W1::tearDown() {}

void G6W1::collect(const messages::SensorData::ConstPtr& msg) {
    int type = getSensorId(msg->type);
    double risk = msg->risk;
    double batt = msg->batt;
    
    battery.consume(BATT_UNIT);
    if (msg->type == "null" || int32_t(risk) == -1)  throw std::domain_error("risk data out of boundaries");

    /*update battery status for received sensor info*/
    if (msg->type == "env_thermometer") {
        env_ther_batt = batt;
        env_ther_raw = msg->data;
    } else if (msg->type == "barometer") {
        baro_batt = batt;
        baro_raw = msg->data;
    } else if (msg->type == "hygrometer") {
        hygro_batt = batt;
        hygro_raw = msg->data;
    } 

    if (buffer_size[type] < max_size) {
        data_buffer[type].push_back(risk);
        buffer_size[type] = data_buffer[type].size();
        total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>());
    } else {
        data_buffer[type].push_back(risk);
        data_buffer[type].erase(data_buffer[type].begin());//erase the first element to avoid overflow
        lost_packt = true;
    }
}

void G6W1::process(){
    battery.consume(BATT_UNIT * data_buffer.size());
    std::vector<double> current_data;

    for(std::vector<std::list<double>>::iterator it = data_buffer.begin(); it != data_buffer.end(); it++) {
        double el = it->front();
        current_data.push_back(el);
        if(it->size() > 1) it->pop_front();
    }

    weather_status = data_fuse(current_data); // consumes 1 packt per sensor (in the buffers that have packages to data_bufferbe processed)
    for (int i = 0; i < buffer_size.size(); ++i){ // update buffer sizes
        buffer_size[i] = data_buffer[i].size();
    }
    total_buffer_size = std::accumulate(std::begin(buffer_size), std::end(buffer_size), 0, std::plus<int>()); //update total buffer size 

    // std::vector<std::string> risks;
    getWeatherStatus();

    std::string weather_risk;

    if(weather_status <= 20) {
        weather_risk = "VERY LOW RISK";
    } else if(weather_status > 20 && weather_status <= 40) {
        weather_risk = "LOW RISK";
    } else if(weather_status > 40 && weather_status <= 60) {
        weather_risk = "MODERATE RISK";
    } else if(weather_status > 60 && weather_status <= 80) {
        weather_risk = "CRITICAL RISK";
    } else if(weather_status > 80 && weather_status <= 100) {
        weather_risk = "VERY CRITICAL RISK";
    }

    std::cout << std::endl << "*****************************************" << std::endl;
    std::cout << "WeatherStatusInfo#" << std::endl;
    std::cout << "| ENVTEMP_RISK: " << env_ther_risk << std::endl;
    std::cout << "| BARO_RISK: " << baro_risk << std::endl;
    std::cout << "| HYGRO_RISK: " << hygro_risk << std::endl;
    std::cout << "| WEATHER_STATE:" << weather_risk << std::endl;
    std::cout << "*****************************************" << std::endl;
}

int32_t G6W1::getSensorId(std::string type) {
    if (type == "env_thermometer")
        return 0;
    else if (type == "barometer")
        return 1;
    else if (type == "hygrometer")
        return 2;
    else {
        std::cout << "UNKNOWN TYPE " + type << std::endl;
        return -1;
    }
}

void G4T1::transfer() {
    messages::TargetSystemData msg;

    msg.env_ther_batt = env_ther_batt;
    msg.baro_batt = baro_batt;
    msg.hygro_batt = hygro_batt;

    msg.env_ther_risk = env_ther_risk;
    msg.baro_risk = baro_risk;
    msg.hygro_risk = hygro_risk;
    
    msg.env_ther_data = env_ther_raw;
    msg.baro_data = baro_raw;
    msg.hygro_data = hygro_raw;

    msg.weather_status = weather_status;

    pub.publish(msg);

    if (lost_packt) {
        lost_packt = false;
        throw std::domain_error("lost data due to package overflow");
    }
}