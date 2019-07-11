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

    size_t uint8_size = sizeof(uint8_t);
    size_t uint16_size = sizeof(uint16_t);
    size_t uint32_size = sizeof(uint32_t);
    size_t int16_size = sizeof(int16_t);
    size_t int32_size = sizeof(int32_t);

    double max_velocity = 1;

    ActionServer *action_server(nullptr); // Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器

    MotorDriver *motorDriver = new MotorDriver("127.0.0.1", 1502, 1, "joint"); // 机械臂驱动
    RTPublisher rt_pub(*motorDriver, "joint_states"); // 关节状态发布器
    traj_follower = new TrajectoryFollower(*motorDriver); // 关节轨迹跟随器
    action_server = new ActionServer(*traj_follower, *motorDriver, "mantra/follow_joint_trajectory", max_velocity); // Action服务器

    TimePoint current_ns = Clock::now();
    uint32_t dur_time;
    for (;;) {
        dur_time = duration_cast<microseconds>(Clock::now() - current_ns).count();
        if (dur_time > 1000000) { // 0.1s
            current_ns = Clock::now();
            cout << "[INFO] time: " << dur_time << endl;

            // 与驱动器通信
            motorDriver->spin_once();
            // 开启Action服务器
            action_server->start();
            // 发布关节状态
//            rt_pub.publish();
            // ROS spin
            ros::spinOnce();
        }
    }

    ros::spin();

    return 0;
}

