//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"
#include "action_server.h"
#include "jointstate_publisher.h"

using namespace Mantra;

int main(int argc, char*argv[]) {

    ros::init(argc, argv, "mantra_driver"); // ROS初始化
    ros::NodeHandle nh("~"); // 节点句柄

    // 获取参数
    bool with_hand = false;
    nh.param("with_hand", with_hand, false);
    if (with_hand) printf("\033[1;32m[INFO] Bring up mantra with gripper!\033[0m\n");

    /// 机械臂驱动、Action服务及轨迹跟踪器实例化
    auto *armDriver = new MotorDriver("192.168.0.1", 502, 1, "joint"); // 机械臂驱动
    Arm::ActionServerArm *action_server_arm(nullptr); // 机械臂Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器
    traj_follower = new TrajectoryFollower(*armDriver); // 关节轨迹跟随器
    action_server_arm = new Arm::ActionServerArm(*traj_follower, *armDriver, "mantra/arm"); // 机械臂Action服务器
    // 开启机械臂Action服务器
    action_server_arm->start();

    /// 手抓驱动及Action服务实例化
    ocservo::OCServoRS485 *handDriver;
    RobotStatePublisher *rs_pub;
    JointStatePublisher *js_pub;
    if (with_hand) {
        handDriver = new ocservo::OCServoRS485(1, "ee_joint1"); // 手抓驱动
        Hand::ActionServerHand *action_server_hand(nullptr); // 手抓Action服务器
        action_server_hand = new Hand::ActionServerHand(*handDriver, "gripper"); // 手抓Action服务器
        // 开启手抓Action服务器
        action_server_hand->start();

        rs_pub = new RobotStatePublisher(*armDriver, *handDriver, "joint_states", 50); // 关节状态发布器
        // 关节状态发布定时器
        rs_pub->start();
    } else {
        js_pub = new JointStatePublisher(*armDriver, "joint_states", 50); // 关节状态发布器
        // 关节状态发布定时器
        js_pub->start();
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(100); // 100HZ

    uint32_t cnt_ = 0;
    while(ros::ok())
    {
//        printf("%d\n", cnt_++); // 运行速度测试

        // 读取关节驱动器中关节位置
        armDriver->do_read_operation();
        // 读当前关节位置, 并写目标位置(轨迹)
        traj_follower->spinOnce();
        // 目标位置发送给关节驱动器
        armDriver->do_write_operation(); // FIXME:读取非连续区域耗时较长, 无法达到100HZ, 实际60HZ左右
        // 更新轨迹执行状态
        action_server_arm->spinOnce();

        loop_rate.sleep();
        ros::spinOnce();
    }

    spinner.stop();
    ROS_INFO("Exiting mantra_driver...");

    return 0;
}

