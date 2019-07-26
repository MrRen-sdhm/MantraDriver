//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"
#include "action_server.h"
#include "robot_state_publisher.h"

// zero_position [ 10.2203 -0.1858 0.0034 0.9514 -0.1892 8.6554 -6.2250 ] [-10.6447 0.1948 54.5139 -10.8382 495.9174 -356.6637 -0.0034 ]

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
    rt_pub.start();
    // 开启Action服务器
    action_server->start();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(100); // 100HZ

    uint32_t cnt_ = 0;
    while(ros::ok())
    {
//        printf("%d\n", cnt_++); // 运行速度测试

        // 读取关节驱动器中关节位置
        motorDriver->do_read_operation();
        // 读当前关节位置, 并写目标位置(轨迹)
        traj_follower->spinOnce();
        // 目标位置发送给关节驱动器
        motorDriver->do_write_operation(); // FIXME:读取非连续区域耗时较长, 无法达到100HZ, 实际60HZ左右
        // 更新轨迹执行状态
        action_server->spinOnce();

        loop_rate.sleep();
        ros::spinOnce();
    }

    spinner.stop();
    ROS_INFO("Exiting mantra_driver...");

    return 0;
}

