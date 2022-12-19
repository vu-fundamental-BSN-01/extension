#include "component/Central Hub Weather.hpp"

#include <iostream>

CentralHubWeather::CentralHubWeather(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery) : Component(argc, argv, name), active(active), max_size(20), total_buffer_size(0), buffer_size({0,0,0,0,0,0}), battery(battery), data_buffer({{0},{0},{0},{0},{0},{0}}) {}

CentralHubWeather::~CentralHubWeather() {}

int32_t CentralHubWeather::run() {
	setUp();

    ros::NodeHandle nh;
    ros::Subscriber therSub = nh.subscribe("ther_data", 10, &CentralHubWeather::collect, this);
    ros::Subscriber hygSub = nh.subscribe("hyg_data", 10, &CentralHubWeather::collect, this);
    ros::Subscriber baroSub = nh.subscribe("baro_data", 10, &CentralHubWeather::collect, this);
    ros::Subscriber reconfigSub = nh.subscribe("reconfigure_"+ros::this_node::getName(), 10, &CentralHubWeather::reconfigure, this);

    while(ros::ok()) {
        ros::Rate loop_rate(rosComponentDescriptor.getFreq());

        try {
            body();
        } catch (const std::exception& e) {
            sendStatus("fail");
        }
        loop_rate.sleep();
    }

    return 0;
}

void CentralHubWeather::body() {
    ros::spinOnce(); //calls collect() if there's data in the topics

    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }
    
    if(isActive()) {
        if(total_buffer_size > 0){
            apply_noise();
            process();
            transfer();
            sendStatus("success");
        }
    } else {
        recharge();
        throw std::domain_error("out of charge");
    }
}

void CentralHubWeather::apply_noise() {}


void CentralHubWeather::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    std::vector<std::string> pairs = bsn::utils::split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = bsn::utils::split(action, '=');

        // Why does replicate_collect updates frequency?
        /*if(param[0]=="replicate_collect"){
            rosComponentDescriptor.setFreq(rosComponentDescriptor.getFreq()+stoi(param[1]));
        }*/
        if(param[0]=="freq"){
            //double new_freq = rosComponentDescriptor.getFreq()+stoi(param[1]);
            double new_freq = stod(param[1]);
            rosComponentDescriptor.setFreq(new_freq);
            /*std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl; 
            ROS_INFO("Calculated New Frequency: [%s]", std::to_string(new_freq).c_str());
            ROS_INFO("New Frequency: [%s]", std::to_string(rosComponentDescriptor.getFreq()).c_str());
            std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;*/
        }
    }
}

bool CentralHubWeather::isActive() {
    return active;
}

void CentralHubWeather::turnOn() {
    active = true;
    activate();
}

void CentralHubWeather::turnOff() {
    active = false;
    deactivate();
}

/*  battery will always recover in 20seconds
    *
    *  b/s = 100% / 20 seconds = 5 %/s 
    *      => recovers 5% battery per second
    *  if we divide by the execution frequency
    *  we get the amount of battery we need to
    *  recover per execution cycle to achieve the
    *  5 %/s battery recovery rate
    */
void CentralHubWeather::recharge() {
    if(battery.getCurrentLevel() <= 100) 
        battery.generate((100.0/20.0)/rosComponentDescriptor.getFreq());
}
