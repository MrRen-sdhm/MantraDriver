//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"
#include "action_server.h"
#include "robot_state_publisher.h"

using namespace Mantra;

int main(int argc, char*argv[]){

    ros::init(argc, argv, "mantra_driver"); // ROS初始化
    ros::NodeHandle nh; // 节点句柄

    size_t uint8_size = sizeof(uint8_t);
    size_t uint16_size = sizeof(uint16_t);
    size_t uint32_size = sizeof(uint32_t);
    size_t int16_size = sizeof(int16_t);
    size_t int32_size = sizeof(int32_t);

    double max_velocity = 1;

    ActionServer *action_server(nullptr); // Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器

    MotorDriver *motorDriver = new MotorDriver("127.0.0.1", 1503, 1, "joint"); // 机械臂驱动
    RTPublisher rt_pub(*motorDriver, "joint_states"); // 关节状态发布器
    traj_follower = new TrajectoryFollower(*motorDriver); // 关节轨迹跟随器
    action_server = new ActionServer(*traj_follower, *motorDriver, "mantra/follow_joint_trajectory", max_velocity); // Action服务器

    // 启动通信定时器
    motorDriver->start();
    // 开启Action服务器
    action_server->start();

    ros::Rate loop_rate(100); // 100HZ
    while(ros::ok())
    {
        // 发布关节状态
//        rt_pub.publish();

        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_WARN("Exiting mantra_driver...");

    return 0;
}

