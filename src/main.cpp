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

    double max_velocity = 1;

    ActionServer *action_server(nullptr); // Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器

    MotorDriver *motorDriver = new MotorDriver("192.168.0.1", 502, 1, "joint"); // 机械臂驱动
    RTPublisher rt_pub(*motorDriver, "joint_states", 50); // 关节状态发布器
    traj_follower = new TrajectoryFollower(*motorDriver); // 关节轨迹跟随器
    action_server = new ActionServer(*traj_follower, *motorDriver, "mantra/follow_joint_trajectory", max_velocity); // Action服务器

    // 关节状态发布定时器
//    rt_pub.start();
    // 开启Action服务器
//    action_server->start();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(10); // 100HZ
    while(ros::ok())
    {
        // 读取关节驱动器中关节位置
        motorDriver->do_read_operation();
        // 读当前关节位置, 并写目标位置(轨迹)
//        traj_follower->spinOnce();
        // 目标位置发送给关节驱动器
        motorDriver->do_write_operation();
        // 更新轨迹执行状态
//        action_server->spinOnce();

        loop_rate.sleep();
        ros::spinOnce();
    }

    spinner.stop();
    ROS_INFO("Exiting mantra_driver...");

    return 0;
}

