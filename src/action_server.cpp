#include <utility>

//
// Created by sdhm on 7/7/19.
//

#include "action_server.h"

namespace Mantra {
ActionServer::ActionServer(TrajectoryFollower &follower, MotorDriver& driver, string action_ns, double max_velocity)
        : as_(nh_, std::move(action_ns), boost::bind(&ActionServer::onGoal, this, _1),
              boost::bind(&ActionServer::onCancel, this, _1), false), driver_(driver), max_velocity_(max_velocity),
          interrupt_traj_(false), has_goal_(false), running_(false), follower_(follower) {
    joint_names_ = driver_.joint_names_;
    std::copy(joint_names_.begin(), joint_names_.end(), std::inserter(joint_set_, joint_set_.end())); // joint_set初始化
}

// 开启Action服务, 并开启轨迹线程
void ActionServer::start() {
    if (running_)
        return;

    ROS_INFO("Starting ActionServer");
    running_ = true;
    tj_thread_ = std::thread(&ActionServer::trajectoryThread, this);
    as_.start();
}

// TODO:机械臂状态更新
void ActionServer::onRobotStateChange() {

    // don't interrupt if everything is fine
//    if (state == RobotState::Running)
//        return;

    // don't retry interrupts
    if (interrupt_traj_ || !has_goal_)
        return;

    // on successful lock we're not executing a goal so don't interrupt
    if (tj_mutex_.try_lock()) {
        tj_mutex_.unlock();
        return;
    }

    interrupt_traj_ = true;
    // wait for goal to be interrupted and automagically unlock when going out of scope
    std::lock_guard<std::mutex> lock(tj_mutex_);

    Result res;
    res.error_code = -100;
    res.error_string = "Robot safety stop";
    curr_gh_.setAborted(res, res.error_string);
}

// 更新机械臂状态数据
bool ActionServer::getCurrState() {
    curr_pos_ = driver_.curr_pos;
    curr_vel_ = driver_.curr_vel;
    return true;
}

// NOTE：接收Action目标的回调函数, 在收到客户端请求时回调
void ActionServer::onGoal(GoalHandle gh) {
    Result res;
    res.error_code = -100;

    ROS_INFO("Received new goal");

    if (!validate(gh, res) || !try_execute(gh, res)) {
        ROS_ERROR("Goal error: %s", res.error_string.c_str());
        gh.setRejected(res, res.error_string);
    }
}

// 取消动作
void ActionServer::onCancel(GoalHandle gh) {
    interrupt_traj_ = true;
    // wait for goal to be interrupted
    std::lock_guard<std::mutex> lock(tj_mutex_);

    Result res;
    res.error_code = -100;
    res.error_string = "Goal cancelled by client";
    gh.setCanceled(res);
}

// 检查机械臂
bool ActionServer::validate(GoalHandle &gh, Result &res) {
    return validateState(gh, res) && validateJoints(gh, res) && validateTrajectory(gh, res);
}

// TODO:检查机械臂状态
bool ActionServer::validateState(GoalHandle &gh, Result &res) {
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
        return true;
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
    return false;
}

// 检查轨迹
bool ActionServer::validateTrajectory(GoalHandle &gh, Result &res) {
    auto goal = gh.getGoal();
    res.error_code = Result::INVALID_GOAL;

    // must at least have one point
    if (goal->trajectory.points.empty())
        return false;

    for (auto const &point : goal->trajectory.points) {
        if (point.velocities.size() != joint_names_.size()) {
            res.error_code = Result::INVALID_GOAL;
            res.error_string = "Received a goal with an invalid number of velocities";
            return false;
        }

        if (point.positions.size() != joint_names_.size()) {
            res.error_code = Result::INVALID_GOAL;
            res.error_string = "Received a goal with an invalid number of positions";
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
                return false;
            }
        }
        for (auto const &position : point.positions) {
            if (!std::isfinite(position)) {
                res.error_string = "Received a goal with infinities or NaNs in positions";
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

// 执行动作
bool ActionServer::try_execute(GoalHandle &gh, Result &res) {
    if (!running_) {
        res.error_string = "Internal error";
        return false;
    }
    if (!tj_mutex_.try_lock()) { // 轨迹线程未上锁
        interrupt_traj_ = true;
        res.error_string = "Received another trajectory";
        curr_gh_.setAborted(res, res.error_string);
        tj_mutex_.lock(); // 上锁
        // todo: make configurable
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    // locked here
    curr_gh_ = gh; // NOTE: 保存当前目标
    interrupt_traj_ = false;
    has_goal_ = true;
    tj_mutex_.unlock(); // 解锁
    tj_cv_.notify_one(); // 唤醒一个线程
    return true;
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

// 轨迹线程
void ActionServer::trajectoryThread() {
    ROS_INFO("Trajectory thread started");

    while (running_) {
        std::unique_lock<std::mutex> lk(tj_mutex_); // 互斥锁
        // 阻塞当前线程
        if (!tj_cv_.wait_for(lk, std::chrono::milliseconds(100), [&] { return running_ && has_goal_; }))
            continue;

        ROS_INFO("Trajectory received and accepted");
        // 检查到轨迹, 开始处理
        curr_gh_.setAccepted();

        auto goal = curr_gh_.getGoal(); // 获取轨迹
        std::vector<TrajectoryPoint> trajectory; /// 待处理轨迹列表
        trajectory.reserve(goal->trajectory.points.size() + 1); // 申请空间

        // moveit发送的 joint_names 可能没有顺序, 重新排序
        auto mapping = reorderMap(goal->trajectory.joint_names);

        ROS_INFO("Translating trajectory");

        auto const &fp = goal->trajectory.points[0]; // 第一个轨迹点
        auto fpt = convert(fp.time_from_start); // 转换为微秒

        // make sure we have a proper t0 position
        if (fpt > std::chrono::microseconds(0)) { // 保证有合适的起始轨迹点
            ROS_INFO("Trajectory without t0 recieved, inserting t0 at currrent position");
            getCurrState(); // 获取当前位置及速度, 作为起始轨迹点
            trajectory.emplace_back(TrajectoryPoint(curr_pos_, curr_vel_, std::chrono::microseconds(0)));
        }

        for (auto const &point : goal->trajectory.points) { /// 处理从moveit接收到的轨迹
            std::array<double, MotorDriver::motor_cnt_> pos{}, vel{};
            for (size_t i = 0; i < MotorDriver::motor_cnt_; i++) {
                size_t idx = mapping[i];
                pos[idx] = point.positions[i];
                vel[idx] = point.velocities[i];
            }
            auto t = convert(point.time_from_start);
            trajectory.emplace_back(TrajectoryPoint(pos, vel, t)); /// moveit生成的轨迹点存入轨迹列表
        }

        double t = std::chrono::duration_cast<std::chrono::duration<double>>( // 末尾轨迹点对应时间
                        trajectory[trajectory.size() - 1].time_from_start).count();
        ROS_INFO("Executing trajectory with %zu points and duration of %4.3fs", trajectory.size(), t);

        Result res;

        ROS_INFO("Attempting to start follower %p", &follower_);
        // 开启轨迹跟踪
        if (follower_.start()) { // 启动轨迹跟踪, 此处running_置true
            if (follower_.execute(trajectory, interrupt_traj_)) { /// 执行轨迹列表中的轨迹
                // interrupted goals must be handled by interrupt trigger
                if (!interrupt_traj_) {
                    ROS_INFO("Trajectory executed successfully");
                    res.error_code = Result::SUCCESSFUL;
                    curr_gh_.setSucceeded(res);
                } else
                    ROS_INFO("Trajectory interrupted");
            } else {
                ROS_INFO("Trajectory failed");
                res.error_code = -100;
                res.error_string = "Connection to robot was lost";
                curr_gh_.setAborted(res, res.error_string);
            }
            follower_.stop(); // 停止轨迹跟踪 此处running_置false
        } else {
            ROS_ERROR("Failed to start trajectory follower!");
            res.error_code = -100;
            res.error_string = "Robot connection could not be established";
            curr_gh_.setAborted(res, res.error_string);
        }

        has_goal_ = false;
        lk.unlock(); // 关闭互斥锁
    }
}

}
