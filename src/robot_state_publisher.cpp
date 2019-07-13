//
// Created by sdhm on 7/7/19.
//

#include "robot_state_publisher.h"

namespace Mantra {
bool RTPublisher::publishJoints(Time &t) {
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = t;

    joint_msg.name.assign(driver_.joint_names_.begin(), driver_.joint_names_.end());
    joint_msg.position.assign(driver_.curr_pos.begin(), driver_.curr_pos.end());
    joint_msg.velocity.assign(driver_.curr_vel.begin(), driver_.curr_vel.end());
    joint_msg.effort.assign(driver_.curr_eff.begin(), driver_.curr_eff.end());

    joint_pub_.publish(joint_msg);

    return true;
}

// FIXME:发布执行器状态
bool RTPublisher::publishTool(Time &t) {
//  geometry_msgs::TwistStamped tool_twist;
//  tool_twist.header.stamp = t;
//  tool_twist.header.frame_id = base_frame_;
//  tool_twist.twist.linear.x = packet.tcp_speed_actual.position.x;
//  tool_twist.twist.linear.y = packet.tcp_speed_actual.position.y;
//  tool_twist.twist.linear.z = packet.tcp_speed_actual.position.z;
//  tool_twist.twist.angular.x = packet.tcp_speed_actual.rotation.x;
//  tool_twist.twist.angular.y = packet.tcp_speed_actual.rotation.y;
//  tool_twist.twist.angular.z = packet.tcp_speed_actual.rotation.z;

//  tool_vel_pub_.publish(tool_twist);
    return true;
}


void RTPublisher::start() {
    // 通信定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_span_), &RTPublisher::pub_callback, this); // 50HZ 受限于主循环频率
    timer_.start();
}

// 定时发布回调函数
void RTPublisher::pub_callback(const ros::TimerEvent& e) {
    Time time = Time::now();
    // 写各关节目标位置
    publishJoints(time);
}

}
