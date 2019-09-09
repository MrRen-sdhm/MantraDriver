//
// Created by sdhm on 7/7/19.
//

#include <utility>
#include "action_server.h"
#include "trajectory_follower.h"

namespace Mantra {
ActionServer::ActionServer(TrajectoryFollower &follower, MotorDriver& driver, string action_ns, double max_velocity)
        : as_(nh_, std::move(action_ns), boost::bind(&ActionServer::onGoal, this, _1),
          boost::bind(&ActionServer::onCancel, this, _1), false), driver_(driver), max_velocity_(max_velocity),
          running_(false), follower_(follower) {
    joint_names_ = driver_.joint_names_;
    std::copy(joint_names_.begin(), joint_names_.end(), std::inserter(joint_set_, joint_set_.end())); // joint_set初始化
}

// 开启Action服务, 并开启轨迹线程
void ActionServer::start() {
    printf("[INFO] Starting ActionServer...\n");
    as_.start();
}

// 接收Action目标的回调函数, 在收到客户端请求时回调
void ActionServer::onGoal(GoalHandle gh) {
    running_ = true; // 开始Action服务
    driver_.emergency_stop_flag_ = false; // 清急停标志

    Result res;
    res.error_code = -100;
    res.error_string = "set goal failed.";

    printf("\033[0;36m[INFO] Action server received a new goal.\033[0m\n");

    if (!validate(gh, res)) { // 机器人状态或轨迹存在问题
        ROS_ERROR("Goal error: %s", res.error_string.c_str());
        gh.setRejected(res, res.error_string);
    } else { // 无错误, 开始轨迹跟踪
        int ret = follower_.set_goal(gh); /// 开始轨迹跟踪
        if (ret) {
            gh.setAccepted();
        } else {
            gh.setRejected(res, res.error_string);
        }
    }
    curr_gh_ = gh; // 获取目标句柄
}

// 轨迹执行状态更新
void ActionServer::spinOnce() {
    Result res;
    if (running_) {
        if (follower_.follow_done_) { // 轨迹跟踪已执行完成, Action仍未完成
            running_ = false; // 结束此次Action
            printf("\033[0;36m[TRAJ] Trajectory executed successfully!\033[0m\n");
            res.error_code = Result::SUCCESSFUL;
            curr_gh_.setSucceeded(res);
            running_ = false; // Action完成, 恢复初始状态
            follower_.follow_done_ = false; // 轨迹跟踪进入准备状态
        }

        if (!driver_.power_on_flag_) { // 运行时关节非使能, 目前作急停使用
            running_ = false; // 结束此次Action
            follower_.cancel(); // 客户端取消轨迹跟踪, 这里为HMI

            printf("\033[0;32m[TRAJ] Goal cancelled by hmi poweroff.\033[0m\n");
            res.error_code = -100;
            res.error_string = "Goal cancelled by hmi poweroff.";
            curr_gh_.setCanceled(res);
        }

        if (driver_.emergency_stop_flag_) { // hmi急停
            running_ = false; // 结束此次Action
            follower_.cancel(); // 客户端取消轨迹跟踪, 这里为HMI

            printf("\033[0;32m[TRAJ] Goal cancelled by hmi emergency stop.\033[0m\n");
            res.error_code = -100;
            res.error_string = "Goal cancelled by hmi emergency stop.";
            curr_gh_.setCanceled(res);

            sleep(1); // 会接收到多次急停指令, 过一会再清标志
            driver_.emergency_stop_flag_ = false;
        }

        if (follower_.path_tolerance_violated) { // 轨迹执行超过容差
            running_ = false; // 结束此次Action
            res.error_code = Result::PATH_TOLERANCE_VIOLATED;
            res.error_string = "PATH_TOLERANCE_VIOLATED";
            curr_gh_.setCanceled(res);
            follower_.path_tolerance_violated = false;
        }

        if (follower_.path_tolerance_violated) { // 终点位置超过容差
            running_ = false; // 结束此次Action
            res.error_code = Result::GOAL_TOLERANCE_VIOLATED;
            res.error_string = "GOAL_TOLERANCE_VIOLATED";
            curr_gh_.setCanceled(res);
            follower_.goal_tolerance_violated = false;
        }
    }
}

// 取消Action动作
void ActionServer::onCancel(GoalHandle gh) {
    running_ = false; // 结束此次Action
    follower_.cancel(); // 客户端取消轨迹跟踪, 即PC端moveit

    Result res;
    res.error_code = -100;
    res.error_string = "Goal cancelled by moveit client.";
    gh.setCanceled(res);
    printf("\033[0;32m\n[TRAJ] Goal cancelled by moveit client.\033[0m\n");
}

// 检查机械臂
bool ActionServer::validate(GoalHandle &gh, Result &res) {
    return validateState(gh, res) && validateJoints(gh, res) && validateTrajectory(gh, res);
}

// TODO:检查机械臂状态
bool ActionServer::validateState(GoalHandle &gh, Result &res) {
    if (!driver_.power_on_flag_) {
        ROS_ERROR("Robot is not power on!");
        return false;
    } else return true;
//    switch (state_) {
//        case RobotState::EmergencyStopped:
//            res.error_string = "Robot is emergency stopped";
//            return false;
//
//        case RobotState::ProtectiveStopped:
//            res.error_string = "Robot is protective stopped";
//            return false;
//
//        case RobotState::Error:
//            res.error_string = "Robot is not ready, check robot_mode";
//            return false;
//
//        case RobotState::Running:
//            return true;
//
//        default:
//            res.error_string = "Undefined state";
//            return false;
//    }
}

// 检查可用关节
bool ActionServer::validateJoints(GoalHandle &gh, Result &res) {
    auto goal = gh.getGoal();
    auto const &joints = goal->trajectory.joint_names;
    std::set<std::string> goal_joints(joints.begin(), joints.end());

    if (goal_joints == joint_set_)
        return true;

    res.error_code = Result::INVALID_JOINTS;
    res.error_string = "Invalid joint names for goal\n";
    res.error_string += "Expected: ";
    std::for_each(goal_joints.begin(), goal_joints.end(),
                  [&res](std::string joint) { res.error_string += joint + ", "; });
    res.error_string += "\nFound: ";
    std::for_each(joint_set_.begin(), joint_set_.end(),
                  [&res](std::string joint) { res.error_string += joint + ", "; });
    printf("joints\n");
    return false;
}

// 检查轨迹
bool ActionServer::validateTrajectory(GoalHandle &gh, Result &res) {
    auto goal = gh.getGoal();
    res.error_code = Result::INVALID_GOAL;

    // 须含有轨迹点
    if (goal->trajectory.points.empty())
        return false;

    for (auto const &point : goal->trajectory.points) {
        if (point.velocities.size() != joint_names_.size()) {
            res.error_code = Result::INVALID_GOAL;
            res.error_string = "Received a goal with an invalid number of velocities";
            ROS_ERROR("Received a goal with an invalid number of velocities");
            return false;
        }

        if (point.positions.size() != joint_names_.size()) {
            res.error_code = Result::INVALID_GOAL;
            res.error_string = "Received a goal with an invalid number of positions";
            ROS_ERROR("Received a goal with an invalid number of positions");
            return false;
        }

        for (auto const &velocity : point.velocities) {
            if (!std::isfinite(velocity)) {
                res.error_string = "Received a goal with infinities or NaNs in velocity";
                return false;
            }
            if (std::fabs(velocity) > max_velocity_) {
                res.error_string =
                        "Received a goal with velocities that are higher than max_velocity_ " +
                        std::to_string(max_velocity_);
                ROS_ERROR("Received a goal with velocities that are higher than max_velocity");
                return false;
            }
        }
        for (auto const &position : point.positions) {
            if (!std::isfinite(position)) {
                res.error_string = "Received a goal with infinities or NaNs in positions";
                ROS_ERROR("Received a goal with infinities or NaNs in positions");
                return false;
            }
        }
    }
    return true;
}

// 时间单位转换
inline std::chrono::microseconds convert(const ros::Duration &dur) {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds(dur.sec) +
                                                                 std::chrono::nanoseconds(dur.nsec));
}

std::vector<size_t> ActionServer::reorderMap(const std::vector<std::string>& goal_joints) {
    std::vector<size_t> indices;
    for (auto const &aj : joint_names_) {
        size_t j = 0;
        for (auto const &gj : goal_joints) {
            if (aj == gj)
                break;
            j++;
        }
        indices.push_back(j);
    }
    return indices;
}

}
