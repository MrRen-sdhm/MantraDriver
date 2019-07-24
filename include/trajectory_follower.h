//
// Created by sdhm on 7/12/19.
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

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "motor_driver.h"
#include "trajectory.h"

using namespace ros;
using namespace std::chrono;

namespace Mantra {

enum class Result_My {
    NONE,      // 未知
    SUCCESS,   // 成功
    FAILED,    // 失败
    CANCLE,    // 取消
};

class TrajectoryFollower {

public:
    typedef control_msgs::FollowJointTrajectoryAction Action;
    typedef control_msgs::FollowJointTrajectoryResult Result;
    typedef actionlib::ServerGoalHandle<Action> GoalHandle;

    explicit TrajectoryFollower(MotorDriver &motorDriver) : motorDriver_(motorDriver){};

    MotorDriver& motorDriver_;

    static const int joint_count_ = MotorDriver::motor_cnt_;
    static const int max_traj_points_ = 200;

    // 两个轨迹点之间的最大位移
    const float max_delta_position_ = 30 * float(M_PI) / 180;

    // 默认容差值, 小于0表示无限制
    JointTolerance <joint_count_> default_path_tol_{};
    JointTolerance <joint_count_> default_goal_tol_{};

    // 当前执行的轨迹
    Trajectory <joint_count_, max_traj_points_> current_trajectory_;

    enum class State {
        STARTING,   // 准备启动
        WORKING,    // 正在跟踪
    };

    /* 以下变量同时在中断和主函数中使用 */
    volatile State state_ = State::STARTING; // 当前工作状态
    volatile bool runing_ = false;           // 是否跟随轨迹
    volatile bool new_read_ = false;         // 有新的反馈数据
    volatile bool new_result_ = false;       // 有新的结果

    TrajectoryPoint <joint_count_> desired_point_{};       // 当前设定的轨迹点
    TrajectoryPoint <joint_count_> actual_point_{};        // 当前实际的轨迹点
    TrajectoryPoint <joint_count_> error_point_{};         // 当前实际的轨迹点

    /* 轨迹跟踪状态 */
    TimePoint time_start_; // 轨迹跟踪的开始时间, 单片机时间
    size_t next_point_index_{}; // 当前插值的下一个轨迹点
    double last_print_time_ = 0; // 轨迹执行状态打印时间

    control_msgs::FollowJointTrajectoryResult result_; // 轨迹执行结果

    // 线性插值算法
    static int linear_interpolate(const TrajectoryPoint <joint_count_> &begin, const TrajectoryPoint <joint_count_> &end,
                           TrajectoryPoint <joint_count_> &current) {
        // 检查时间戳
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            return -1;
        }
        float percent = float(current.time_nsec - begin.time_nsec) / (end.time_nsec - begin.time_nsec);
        for (int i = 0; i < joint_count_; i++) {
            float delta_pos = end.positions[i] - begin.positions[i];
            float pos = begin.positions[i] + percent * delta_pos;
            current.positions[i] = pos;
        }
        return 0;
    }

//    void on_emergency_stop_changed(bool value) override {
//        if (value) {
//            arm_.enable_motor(0, 0);
//            InterruptLock lock;
//            std::unique_lock<InterruptLock> lock_guard(lock);
//            enable_motor_ = false;
//            desired_point_ = actual_point_;
//            desired_point_.time_nsec = 0;
//            lock_guard.unlock();
//            _finish_with_error(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
//                               "emergency_stop_");
//
//            printf("%s: emergency_stop_!!!\n", name_);
//        } else {
//            InterruptLock lock;
//            std::unique_lock<InterruptLock> lock_guard(lock);
//            desired_point_ = actual_point_;
//            desired_point_.time_nsec = 0;
//            lock_guard.unlock();
//
//            printf("%s: emergency_stop_ released\n", name_);
//        }
//    }

    // 设置关节电机目标位姿
    void _set_motor_positions(std::array<float, joint_count_> &positions) {
        for (int id = 1; id <= joint_count_; id++) {
            motorDriver_.set_position(id, positions[id-1]);
        }
    }

    // 对比actual与desired, 计算误差error_point_
    void _compute_error(const TrajectoryPoint <joint_count_> &desired, const TrajectoryPoint <joint_count_> &actual,
                        TrajectoryPoint <joint_count_> &) {
        for (int i = 0; i < joint_count_; i++) {
            float error = desired.positions[i] - actual.positions[i];
            error_point_.positions[i] = error;
        }
        error_point_.time_nsec = std::max(desired.time_nsec, actual.time_nsec);
    }

    // 检查误差是否在容许范围内
    static bool _is_error_in_tolerance(const TrajectoryPoint <joint_count_> &error,
                                       const JointTolerance <joint_count_> &tolerance, bool display = false) {
        for (int i = 0; i < joint_count_; i++) {
            if (std::abs(error.positions[i]) > tolerance.positions[i]) {
                if (display) {
                    printf("joint %d: abs(error %f) > tol %f\n", i + 1, R2D(std::abs(error.positions[i])),
                           R2D(tolerance.positions[i]));
                }
                return false;
            }
        }
        return true;
    }

    // 以给定错误停止任务执行
    void _finish_with_error(int32_t error_code, const char *error_string) {
        if (runing_) {
            // 记录执行结果
            result_.error_code = error_code;
            result_.error_string = error_string;
            new_result_ = true;

            // 关节保持当前实际位置
            runing_ = false;
        }
    }

    // 检查位置与时间容差, 若超出容差则中止执行, 若达到目标则完成执行
//    void _check_tolerance() {
//        uint32_t now = Time::now().nsec;
//        // 计算容差
//        _compute_error(desired_point_, actual_point_, error_point_);
//        if (next_point_index_ < current_trajectory_.points_length) {
//            // 正在执行
//            if (!_is_error_in_tolerance(error_point_, current_trajectory_.path_tol, true)) {
//                // 误差超过容许范围!, 结束执行
//                desired_point_ = actual_point_;
//                desired_point_.time_nsec = 0;
//                _finish_with_error(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
//                                   "PATH_TOLERANCE_VIOLATED");
//                printf("[Result]: PATH_TOLERANCE_VIOLATED\n");
//            } else {
//                // 误差在容许范围内, 继续执行
//            }
//        } else {
//            // 等待结束
//            if (_is_error_in_tolerance(error_point_, current_trajectory_.goal_tol)) {
//                // 误差在容许范围, 结束执行
//
//                // 记录执行结果
//                result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
//                result_.error_string = "";
//                new_result_ = true;
//
//                /// 返回到空闲状态
//                runing_ = false;
//
//                auto &p = error_point_;
//                printf("[Result]: SUCCESSFUL error=[");
//                for (int i = 0; i < joint_count_; i++) {
//                    if (i > 0)
//                        printf(",");
//                    printf("%.2f", R2D(p.positions[i]));
//                }
//                printf("]\n");
//            } else {
//                // 误差超过容许范围
//                if (now - time_start_ <=
//                    current_trajectory_.points[current_trajectory_.points_length - 1].time_nsec +
//                    current_trajectory_.goal_time_tolerance) {
//                    // 用时未超过容差范围, 继续等待
//                } else {
//                    // 用时超出, 强制停止
//                    _is_error_in_tolerance(error_point_, current_trajectory_.goal_tol, true);
//
//                    _finish_with_error(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED,
//                                       "GOAL_TOLERANCE_VIOLATED");
//                    printf("[Result]: %.3f > %.3f GOAL_TOLERANCE_VIOLATED\n", (now - time_start_) / 1e6f,
//                           (current_trajectory_.points[current_trajectory_.points_length - 1].time_nsec +
//                            current_trajectory_.goal_time_tolerance) / 1e6f);
//                }
//            }
//        }
//    }

    void spinOnce() {
        // 读当前关节位置
        on_read_pos();
        // 写目标关节位置
        on_sync_write();
    }

    // 主循环中写入新位置到虚拟驱动器
    void on_sync_write() {
        if (runing_) {
            uint64_t duration;
            TimePoint now = Clock::now();

            switch (state_) {
                case State::STARTING:
                    time_start_ = now;
                    next_point_index_ = 0;
                    state_ = State::WORKING;
                    printf("\033[0;36m[TRAJ] Start follow trajectory.\033[0m\n");
                    // don't break here

                case State::WORKING:
                    // 计算距离任务开始的时间
                    duration = duration_cast<nanoseconds>(now - time_start_).count();
                    // 找到下一个要跟踪的轨迹点
                    while (duration > current_trajectory_.points[next_point_index_].time_nsec &&
                           next_point_index_ < current_trajectory_.points_length) {
                        next_point_index_++;
                    }

                    // 计算目标位置
                    if (next_point_index_ == 0) {
                        // 起始
                        desired_point_ = current_trajectory_.points[0];
                        // 设定电机状态
                        _set_motor_positions(desired_point_.positions);
                    } else if (next_point_index_ < current_trajectory_.points_length) {
                        // 中间状态, 计算插值点
                        desired_point_.time_nsec = duration;
                        // 线性插值
                        linear_interpolate(current_trajectory_.points[next_point_index_ - 1],
                                               current_trajectory_.points[next_point_index_], desired_point_);
                        // 设定电机状态
                        _set_motor_positions(desired_point_.positions);
                    } else {
                        // 正在结束
                        desired_point_ = current_trajectory_.points[current_trajectory_.points_length - 1];
                        _set_motor_positions(desired_point_.positions);

                        // 轨迹执行完成
                        printf("\033[0;36m[TRAJ] Follow trajectory done.\033[0m\n");
                        runing_ = false;
                        result_.error_string = "SUCCESS";
                        state_ = State::STARTING;
                    }

                    // 打印插补后的轨迹
//                    printf("[Traj] servoj[%lu]([%f,%f,%f,%f,%f,%f,%f])\n", next_point_index_, desired_point_.positions[0],
//                             desired_point_.positions[1], desired_point_.positions[2], desired_point_.positions[3],
//                             desired_point_.positions[4], desired_point_.positions[5], desired_point_.positions[6]);

                    // 打印状态
                    if (Time::now().toSec() - last_print_time_ > 0) { // 间隔打印, 设为0即一直打印
                        last_print_time_ = Time::now().toSec();
                        auto &p = desired_point_;
                        printf("\033[0;36m[TRAJ]\033[0m %zu/%zu %.3fsec [", next_point_index_,
                               current_trajectory_.points_length, duration / 1e9f);
                        for (int i = 0; i < joint_count_; i++) {
                            if (i > 0)
                                printf(",");
                            printf("%.3f(%.3f)", p.positions[i], R2D(p.positions[i]));
                        }
                        printf("]\n");
                    }

                    // 检查容差
    //            _check_tolerance();
            }
        }
    }

    // 当读取到新位置时的回调
    void on_read_pos() {
        // 将电机位置反馈记录到actual_point_
        actual_point_.time_nsec = Time::now().nsec;
        actual_point_.positions = motorDriver_.curr_pos;
        new_read_ = true;

//        if (runing_) _check_tolerance();
    }

    // 设置要跟踪的目标轨迹
    bool set_goal(const GoalHandle& gh) {
//        if (emergency_stop()) {
//            // 急停状态
//            printf("Goal rejected, emergency_stop_\n");
//            result_.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
//            result_.error_string = "emergency_stop_";
//            new_result_ = true;
//            return -4;
//        }

        // 检查轨迹消息
        auto parser = TrajectoryParser<joint_count_, max_traj_points_> (motorDriver_.joint_names_, gh,
                                                                           current_trajectory_);
        // 拷贝容差值
//        parser.copy_tolerance(default_path_tol_, default_goal_tol_);
        // 拷贝轨迹数据
        parser.copy_trajectory();

        printf("\033[0;36m[TRAJ] Goal accepted, start trajectory follow.\033[0m\n");

        current_trajectory_.print();

        runing_ = true; /// 置为启动状态

        return true;
    }

    bool cancle_by_client() {
        printf("\033[0;34m[TRAJ] Goal cancelled by client.\033[0m\n");
        runing_ = false;
    }

//    // 由主循环调用, 取消当前操作
//    int cancel() {
//        // 设定空消息, 即为取消操作
//        printf("Canceled\n");
//        set_goal(GoalHandle);
//        result_.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
//        result_.error_string = "canceled";
//        new_result_ = true;
//        return 0;
//    }

//    // 由主循环调用, 获取当前反馈信息, 当没有新的反馈信息或未启动时, 返回-1
//    int get_feedback(control_msgs::FollowJointTrajectoryFeedback &fb) {
//        InterruptLock lock;
//        InterruptLockGuard lock_guard(lock);
//
//        int ret;
//        if (new_read_) {
//
//            fb.header.seq = arm_.count_read_;
//            fb.header.stamp.fromNSec(arm_.last_recv_ok_);
//            fb.header.frame_id = "";
//            fb.joint_names_length = arm_.joint_names().size();
//            fb.joint_names = (char **) arm_.joint_names().data();
//            fb.actual.positions_length = (uint32_t) joint_count_;
//            fb.actual.positions = actual_point_.positions;
//            fb.desired.positions_length = (uint32_t) joint_count_;
//            fb.desired.positions = desired_point_.positions;
//            fb.error.positions_length = (uint32_t) joint_count_;
//            fb.error.positions = error_point_.positions;
//
//            new_read_ = false;
//            ret = 0;
//        } else {
//            ret = -1;
//        }
//        return ret;
//    }
//
//    // 由主循环调用, 获取结果
//    int get_result(control_msgs::FollowJointTrajectoryResult **result_out) {
//        InterruptLock lock;
//        InterruptLockGuard lock_guard(lock);
//
//        int ret;
//        if (new_result_) {
//            *result_out = &result_;
//            new_result_ = false;
//            ret = 0;
//        } else {
//            *result_out = NULL;
//            ret = -1;
//        }
//
//        return ret;
//    }

};

}