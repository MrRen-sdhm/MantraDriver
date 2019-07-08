//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"
#include "action_server.h"
#include "robot_state_publisher.h"

using namespace Mantra;

int main(int argc, char*argv[]){

    size_t uint8_size = sizeof(uint8_t);
    size_t uint16_size = sizeof(uint16_t);
    size_t uint32_size = sizeof(uint32_t);

    double max_velocity = 1;

    ActionServer *action_server(nullptr); // Action服务器
    TrajectoryFollower *traj_follower; // 关节轨迹跟随器

    MotorDriver *motorDriver = new MotorDriver("127.0.0.1", 1502, 1); // 机械臂驱动
    RTPublisher rt_pub(*motorDriver); // 关节状态发布器
    traj_follower = new TrajectoryFollower(*motorDriver); // 关节轨迹跟随器
    action_server = new ActionServer(*traj_follower, *motorDriver, max_velocity); // Action服务器

    TimePoint current_ns = Clock::now();
    uint32_t dur_time;
    for (;;) {
        dur_time = duration_cast<microseconds>(Clock::now() - current_ns).count();
        if (dur_time > 1000000) {
            cout << "[INFO] time: " << dur_time << endl;
            // 与驱动器通信
            motorDriver->spin_once();
            current_ns = Clock::now();
        }
    }

    return 0;
}

