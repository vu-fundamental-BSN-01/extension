#include "component/g3t1_6/G3T1_6.hpp"

#define BATT_UNIT 0.05
#define INCREMENT 0.1

using namespace bsn::range;
using namespace bsn::resource;
using namespace bsn::generator;
using namespace bsn::configuration;


G3T1_6::G3T1_6(int &argc, char **argv, const std::string &name) :
    Sensor(argc, argv, name, "glucosemeter", true, 1, bsn::resource::Battery("glc_batt", 100, 100, 1), false),
    markov(),
    dataGenerator(),
    filter(1),
    sensorConfig(),
    collected_risk() {}

G3T1_6::~G3T1_6() {}

void G3T1_6::setUp() {
    Component::setUp();
    
    std::array<bsn::range::Range,5> ranges;
    std::string s;

    handle.getParam("start", shouldStart);

    { // Get ranges
        std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

        handle.getParam("glucose_LowRisk", s);
        lrs = bsn::utils::split(s, ',');
        handle.getParam("glucose_MidRisk0", s);
        mrs0 = bsn::utils::split(s, ',');
        handle.getParam("glucose_HighRisk0", s);
        hrs0 = bsn::utils::split(s, ',');
        handle.getParam("glucose_MidRisk1", s);
        mrs1 = bsn::utils::split(s, ',');
        handle.getParam("glucose_HighRisk1", s);
        hrs1 = bsn::utils::split(s, ',');

        ranges[0] = Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
        ranges[1] = Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
        ranges[2] = Range(std::stod(lrs[0]), std::stod(lrs[1]));
        ranges[3] = Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
        ranges[4] = Range(std::stod(hrs1[0]), std::stod(hrs1[1]));
    }

    { // Configure sensor configuration
        Range low_range = ranges[2];
        
        std::array<Range,2> midRanges;
        midRanges[0] = ranges[1];
        midRanges[1] = ranges[3];
        
        std::array<Range,2> highRanges;
        highRanges[0] = ranges[0];
        highRanges[1] = ranges[4];

        std::array<Range,3> percentages;

        handle.getParam("lowrisk", s);
        std::vector<std::string> low_p = bsn::utils::split(s, ',');
        percentages[0] = Range(std::stod(low_p[0]), std::stod(low_p[1]));

        handle.getParam("midrisk", s);
        std::vector<std::string> mid_p = bsn::utils::split(s, ',');
        percentages[1] = Range(std::stod(mid_p[0]), std::stod(mid_p[1]));

        handle.getParam("highrisk", s);
        std::vector<std::string> high_p = bsn::utils::split(s, ',');
        percentages[2] = Range(std::stod(high_p[0]), std::stod(high_p[1]));

        sensorConfig = SensorConfiguration(0, low_range, midRanges, highRanges, percentages);
    }

    { //Check for instant recharge parameter
        handle.getParam("instant_recharge", instant_recharge);
    }
}

void G3T1_6::tearDown() {
    Component::tearDown();
}

double G3T1_6::collect() {
    double m_data = 0;
    ros::ServiceClient client = handle.serviceClient<services::PatientData>("getPatientData");
    services::PatientData srv;

    srv.request.vitalSign = "glucose";

    if (client.call(srv)) {
        m_data = srv.response.data;
        ROS_INFO("new data collected: [%s]", std::to_string(m_data).c_str());
    } else {
        ROS_INFO("error collecting data");
    }

    battery.consume(BATT_UNIT);
    cost += BATT_UNIT;

    collected_risk = sensorConfig.evaluateNumber(m_data);

    return m_data;
}

double G3T1_6::process(const double &m_data) {
    double filtered_data;
    
    
    filter.insert(m_data);
    filtered_data = filter.getValue();
    battery.consume(BATT_UNIT*filter.getRange());
    cost += BATT_UNIT*filter.getRange();

    ROS_INFO("filtered data: [%s]", std::to_string(filtered_data).c_str());
    return filtered_data;
}

void G3T1_6::transfer(const double &m_data) {
    double risk;
    risk = sensorConfig.evaluateNumber(m_data);
    if (risk < 0 || risk > 100) throw std::domain_error("risk data out of boundaries");
    if (label(risk) != label(collected_risk)) throw std::domain_error("sensor accuracy fail");

    ros::NodeHandle handle;
    data_pub = handle.advertise<messages::SensorData>("glucosemeter_data", 10);
    messages::SensorData msg;
    msg.type = type;
    msg.data = m_data;
    msg.risk = risk;
    msg.batt = battery.getCurrentLevel();

    data_pub.publish(msg);
    
    battery.consume(BATT_UNIT);
    cost += BATT_UNIT;

    ROS_INFO("risk calculated and transferred: [%.2f%%]", risk);
}

std::string G3T1_6::label(double &risk) {
    std::string ans;
    if(sensorConfig.isLowRisk(risk)){
        ans = "low";
    } else if (sensorConfig.isMediumRisk(risk)) {
        ans = "moderate";
    } else if (sensorConfig.isHighRisk(risk)) {
        ans = "high";
    } else {
        ans = "unknown";
    }

    return ans;
}
