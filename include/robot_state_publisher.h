//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cstdlib>
#include <vector>

#include "motor_driver.h"

using namespace ros;

namespace Mantra {
class RTPublisher {
public:
    explicit RTPublisher(MotorDriver& driver) : driver_(driver), joint_pub_(
            nh_.advertise<sensor_msgs::JointState>("joint_states", 1)) {
    }

private:
    NodeHandle nh_;
    Publisher joint_pub_;

    MotorDriver& driver_;

    bool publishJoints(Time &t);

    bool publishTool(Time &t);

    bool publish();

};

}