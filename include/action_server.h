//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <thread>
#include <cmath>

#include "motor_driver.h"
#include "trajectory_follower.h"

using namespace ros;

namespace Mantra {
class ActionServer {
private:
    typedef control_msgs::FollowJointTrajectoryAction Action;
    typedef control_msgs::FollowJointTrajectoryResult Result;
    typedef actionlib::ServerGoalHandle<Action> GoalHandle;
    typedef actionlib::ActionServer<Action> Server;

    NodeHandle nh_;
    Server as_;

    std::vector<std::string> joint_names_;
    std::set<std::string> joint_set_;
    double max_velocity_;

    GoalHandle curr_gh_;
    std::atomic<bool> interrupt_traj_;
    std::atomic<bool> has_goal_, running_;
    std::mutex tj_mutex_;
    std::condition_variable tj_cv_;
    std::thread tj_thread_;

    TrajectoryFollower &follower_;

    std::array<double, MotorDriver::motor_cnt_> curr_pos_{}, curr_vel_{};

    void onGoal(GoalHandle gh);

    void onCancel(GoalHandle gh);

    bool validate(GoalHandle &gh, Result &res);

    bool validateState(GoalHandle &gh, Result &res);

    bool validateJoints(GoalHandle &gh, Result &res);

    bool validateTrajectory(GoalHandle &gh, Result &res);

    bool try_execute(GoalHandle &gh, Result &res);

    void interruptGoal(GoalHandle &gh);

    std::vector<size_t> reorderMap(const std::vector<std::string>& goal_joints);

    void trajectoryThread();

    bool getCurrState();

public:
    ActionServer(TrajectoryFollower &follower, MotorDriver& driver, string action_ns, double max_velocity);

    void start();
    MotorDriver& driver_;

    void onRobotStateChange();
};

}