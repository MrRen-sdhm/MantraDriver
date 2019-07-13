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
    explicit RTPublisher(MotorDriver& driver, const string& topic_name, int timer_span) : driver_(driver), joint_pub_(
            nh_.advertise<sensor_msgs::JointState>(topic_name, 1)), timer_span_(timer_span) {
    }

    void start();
    void pub_callback(const ros::TimerEvent& e);

private:
    /// 定时器相关参数
    ros::Timer timer_;
    ros::NodeHandle nh_;
    int timer_span_; // 通信频率 HZ

    Publisher joint_pub_;

    MotorDriver& driver_;

    bool publishJoints(Time &t);

    bool publishTool(Time &t);
};

}