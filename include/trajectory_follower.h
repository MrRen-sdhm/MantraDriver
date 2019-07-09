//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "motor_driver.h"

//#include "action_trajectory_follower_interface.h"

namespace Mantra {
struct TrajectoryPoint {
    std::array<double, Mantra::MotorDriver::motor_cnt_> positions;
    std::array<double, Mantra::MotorDriver::motor_cnt_> velocities;
    std::chrono::microseconds time_from_start;

    TrajectoryPoint() {
    }

    TrajectoryPoint(std::array<double, Mantra::MotorDriver::motor_cnt_> &pos,
                    std::array<double, Mantra::MotorDriver::motor_cnt_> &vel, std::chrono::microseconds tfs)
            : positions(pos), velocities(vel), time_from_start(tfs) {
    }
};

class TrajectoryFollower {
private:
    static const uint8_t joint_cnt_ = Mantra::MotorDriver::motor_cnt_;
    MotorDriver& motorDriver_;
    std::atomic<bool> running_;
    std::array<double, joint_cnt_> last_positions_;
    double servoj_time_;

    bool execute(std::array<double,joint_cnt_> &positions);

    static double interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel);

public:
    TrajectoryFollower(MotorDriver &motorDriver);

    bool start();

    bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);

    void stop();

    virtual ~TrajectoryFollower() {};
};

}