//
// Created by coumarc9 on 7/24/17. and modified by Lucas
//

#ifndef INTERFACE_CONFIGURATION_H
#define INTERFACE_CONFIGURATION_H

#include <cstdint>
#include <cmath>
#include <ros/ros.h>

namespace interface_rs485
{
    class Configuration {

    public:

        Configuration(const ros::NodeHandlePtr &nh);
        ~Configuration();

        std::string getTtyPort() const {return ttyPort;}

    private:

        ros::NodeHandlePtr nh;

        std::string ttyPort;

        void Deserialize();
        void SetParameter();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);


        };
}




#endif //PROVIDER_HYDROPHONE_CONFIGURATION_H
