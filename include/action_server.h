//
// Created by sdhm on 7/7/19.
//

#pragma once

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
    bool running_;

    TrajectoryFollower &follower_;

    void onGoal(GoalHandle gh);

    void onCancel(GoalHandle gh);

    bool validate(GoalHandle &gh, Result &res);

    bool validateState(GoalHandle &gh, Result &res);

    bool validateJoints(GoalHandle &gh, Result &res);

    bool validateTrajectory(GoalHandle &gh, Result &res);

    std::vector<size_t> reorderMap(const std::vector<std::string>& goal_joints);

public:
    ActionServer(TrajectoryFollower &follower, MotorDriver& driver, string action_ns, double max_velocity);

    MotorDriver& driver_;
    void start();
    void spinOnce();
};

}