//
// Created by coumarc9 on 7/24/17. and modified by Lucas ^^
//

#include "Configuration.h"

namespace interface_rs485
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh(nh),
          ttyPort("/dev/RS485"),
          sleepTime(0.1),
          dataReadChunk(8192)
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_INFO("Deserialize params");

        FindParameter("/connection/tty_port", ttyPort);
        FindParameter("/data/sleep_time", sleepTime);
        FindParameter("/data/read_chunk", dataReadChunk);

        ROS_INFO("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh->hasParam("/interface_RS485" + paramName)) {
            nh->getParam("/interface_RS485" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /interface_RS485" + paramName
                                    << ". Using default.");
        }
    }

}
