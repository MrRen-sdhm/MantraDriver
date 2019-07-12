//
// Created by sdhm on 7/7/19.
//

#include "trajectory_follower_ur.h"
#include <endian.h>
#include <ros/ros.h>
#include <cmath>

namespace Mantra {

TrajectoryFollower::TrajectoryFollower(MotorDriver& motorDriver)
        : running_(false), motorDriver_(motorDriver), servoj_time_(0.008){

}

bool TrajectoryFollower::start() {
    if (running_)
        return true;

    ROS_DEBUG("Robot successfully connected");
    return (running_ = true);
}

bool TrajectoryFollower::execute(std::array<double, joint_cnt_> &positions) {
    if (!running_) {
        cout << "!running\n" << endl;
        return false;
    }

    ROS_INFO("servoj[%lu]([%f,%f,%f,%f,%f,%f,%f])", excute_cnt_++, positions[0], positions[1], positions[2],
            positions[3], positions[4], positions[5], positions[6]);

    last_positions_ = positions;

    /// 写目标位置到 Mantra 虚拟寄存器中
    for (uint8_t id = 1; id < joint_cnt_; id++) {
        bool ret = motorDriver_.set_position(id, positions[id-1]);
        if (!ret) return false;
    }

    // FIXME: 此处出现读写冲突！！！
    return motorDriver_.do_write_operation(); /// 写目标位置到实际控制器
    return true;
}

// 轨迹插补
double
TrajectoryFollower::interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel) {
    using std::pow;
    double a = p0_pos;
    double b = p0_vel;
    double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
    double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
    return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}

// 执行轨迹（有轨迹插补）
bool TrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt) {
    if (!running_)
        return false;

    using namespace std::chrono;
    typedef duration<double> double_seconds;
    typedef Clock::time_point Time;

    auto &last = trajectory[trajectory.size() - 1]; // 轨迹数组末尾
    auto &prev = trajectory[0];

    Time t0 = Clock::now();
    Time latest = t0;

    std::array<double, joint_cnt_> positions{};

    for (auto const &point : trajectory) {
        // skip t0
        if (&point == &prev)
            continue;

        if (interrupt)
            break;

        auto duration = point.time_from_start - prev.time_from_start; // 此点与第一点时间差
        double d_s = duration_cast<double_seconds>(duration).count();

        // 轨迹插补
        while (!interrupt) {
            latest = Clock::now();
            auto elapsed = latest - t0;

            if (point.time_from_start <= elapsed)
                break;

            if (last.time_from_start <= elapsed)
                return true;

            double elapsed_s = duration_cast<double_seconds>(elapsed - prev.time_from_start).count();
            for (size_t j = 0; j < positions.size(); j++) {
                /// 轨迹插补
                positions[j] = interpolate(elapsed_s, d_s, prev.positions[j], point.positions[j], prev.velocities[j],
                                    point.velocities[j]);
            }

            if (!execute(positions)) {
                return false;
            }

            /// 线程休眠, 控制插补数量
            std::this_thread::sleep_for(std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.)));
        }

        prev = point;
    }

    // In theory it's possible the last position won't be sent by
    // the interpolation loop above but rather some position between
    // t[N-1] and t[N] where N is the number of trajectory points.
    // To make sure this does not happen the last position is sent
    return execute(last.positions);
}

void TrajectoryFollower::stop() {
    if (!running_)
        return;

    // std::array<double, joint_cnt_> empty;
    // execute(empty, false);

//  server_.disconnectClient(); // NOTE：？？？
    running_ = false;
}

}