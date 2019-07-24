//
// Created by sdhm on 7/12/19.
//

#pragma once

#include <array>
#include <cmath>
#include <utility>
#include <functional>
#include <limits>
#include <utility>

#include <control_msgs/FollowJointTrajectoryGoal.h>

namespace Mantra {

template <size_t joint_count>
struct JointTolerance {
    std::array<float, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
};

// 一个轨迹点
template <size_t joint_count>
struct TrajectoryPoint {
    std::array<float, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
    //        std::array<float, joint_count> effort;
    uint64_t time_nsec; // 相对于轨迹跟踪开始的时间偏移
};

// 轨迹
template <size_t joint_count, size_t max_trajectory_points>
struct Trajectory {
    static const int joint_count_ = joint_count;
    /* 路径容差值, 小于0表示无限制 */
    JointTolerance<joint_count> path_tol;
    /* 最终位置容差值, 小于0表示无限制 */
    JointTolerance<joint_count> goal_tol;
    uint64_t goal_time_tolerance; // 结束时间容差值
    // 待执行的轨迹
    std::array<TrajectoryPoint<joint_count>, max_trajectory_points> points;
    // 待执行的轨迹长度
    size_t points_length = 0;

    uint32_t dummy = 0;

    void print() {
        if (points_length <= 0) {
            printf("[TRAJ Empty]\n");
            return;
        }
        printf("[TRAJ] length=%zu (%.3f s)]\n", points_length, (points[points_length - 1].time_nsec - points[0].time_nsec) / 1e9f);

        printf("[TRAJ] path_tol=[");
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", path_tol.positions[i], R2D(path_tol.positions[i]));
        }
        printf("]\n");

        printf("[TRAJ] goal_tol=[");
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", goal_tol.positions[i], R2D(goal_tol.positions[i]));
        }
        printf("]\n");

        printf("[TRAJ] goal_time_tolerance=%d ms\n", (int)(goal_time_tolerance / 1000000));
        auto& p = points[0];
        printf("[TRAJ] begin (%d ms)=[", int(p.time_nsec / 1000000));
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", p.positions[i], R2D(p.positions[i]));
        }
        printf("]\n");

        auto& p1 = points[points_length - 1];
        printf("[TRAJ] end (%d ms)=[", int(p1.time_nsec / 1000000));
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f(%.2f)", p1.positions[i], R2D(p1.positions[i]));
        }
        printf("]\n");
    }
};

template <size_t joint_count, size_t max_trajectory_points>
class TrajectoryParser {
public:
    typedef control_msgs::FollowJointTrajectoryAction Action;
    typedef control_msgs::FollowJointTrajectoryResult Result;
    typedef actionlib::ServerGoalHandle<Action> GoalHandle;

    std::vector<string> joint_names;

    const GoalHandle& gh;

    Trajectory<joint_count, max_trajectory_points>& traj; // 待执行的轨迹

    TrajectoryParser(std::vector<string>  _joint_names, decltype(gh) _gh, decltype(traj) _traj)
            : joint_names(std::move(_joint_names)), gh(_gh) ,traj(_traj) {
    }

    // 从JointTrajectory消息中拷贝轨迹数据, 并将环形关节的位置设定值限制为 -pi 到 pi
    // id_to_index由_check_trajectory计算得到
    void copy_trajectory() {
        auto goal = gh.getGoal();
        // 拷贝数据
        for (int i = 0; i < goal->trajectory.points.size(); i++) {
            const trajectory_msgs::JointTrajectoryPoint& p = goal->trajectory.points[i];
            // 相对于轨迹跟踪开始的时间偏移 单位ns
            traj.points[i].time_nsec = (uint64_t)p.time_from_start.sec * 1000000000 + p.time_from_start.nsec;

            for (int j = 0; j < joint_count; j++) {
                float pos = p.positions[j];
                traj.points[i].positions[j] = pos;
            }
        }
        traj.points_length = goal->trajectory.points.size();
    }

    // 拷贝容差值
    void _copy_tolerance(const control_msgs::JointTolerance* tols, size_t length, const JointTolerance<joint_count>& default_tols, JointTolerance<joint_count>& tols_out) {
        // 建立关节ID到列表序号的索引
        std::array<int, joint_count> id_to_index;
        std::fill_n(id_to_index.begin(), joint_count, -1);
        for (int i = 0; i < length; i++) {
            for (int id = 1; id <= joint_count; id++) {
                if (tols[i].name == joint_names[id - 1]) {
                    id_to_index[id - 1] = i;
                }
            }
        }
        // 检查并拷贝数据
        for (int i = 0; i < joint_count; i++) {
            if (id_to_index[i] < 0) {
                // 没有给定容差值, 使用默认值
                tols_out.positions[i] = default_tols.positions[i];
            } else {
                const control_msgs::JointTolerance& tol = tols[id_to_index[i]];
                if (tol.position < 0) {
                    // 给定值小于0, 容差值无限制
                    tols_out.positions[i] = -1;
                } else if (tol.position == 0 || !std::isfinite(tol.position)) {
                    // 给定值为0, 或给定值为异常数, 使用默认容差值
                    tols_out.positions[i] = default_tols.positions[i];
                } else {
                    // 使用给定容差值
                    tols_out.positions[i] = tol.position;
                }
            }
        }
    }

    void copy_tolerance(const JointTolerance<joint_count>& default_path_tol, const JointTolerance<joint_count>& default_goal_tol) {
        auto goal = gh.getGoal();
        auto path_tols = goal->path_tolerance;
        auto goal_tols = goal->goal_tolerance;

        // 建立关节ID到列表序号的索引
        std::array<int, joint_count> id_to_index_1;
        std::fill_n(id_to_index_1.begin(), joint_count, -1);
        for (int i = 0; i < path_tols.size(); i++) {
            for (int id = 1; id <= joint_count; id++) {
                if (path_tols[i].name == joint_names[id - 1]) {
                    id_to_index_1[id - 1] = i;
                }
            }
        }
        // 检查并拷贝数据
        for (int i = 0; i < joint_count; i++) {
            if (id_to_index_1[i] < 0) {
                // 没有给定容差值, 使用默认值
                traj.path_tol.positions[i] = default_path_tol.positions[i];
            } else {
                const control_msgs::JointTolerance& tol = goal_tols[id_to_index_1[i]];
                if (tol.position < 0) {
                    // 给定值小于0, 容差值无限制
                    traj.path_tol.positions[i] = -1;
                } else if (tol.position == 0 || !std::isfinite(tol.position)) {
                    // 给定值为0, 或给定值为异常数, 使用默认容差值
                    traj.path_tol.positions[i] = default_path_tol.positions[i];
                } else {
                    // 使用给定容差值
                    traj.path_tol.positions[i] = tol.position;
                }
            }
        }

        // 建立关节ID到列表序号的索引
        std::array<int, joint_count> id_to_index_2;
        std::fill_n(id_to_index_2.begin(), joint_count, -1);
        for (int i = 0; i < goal_tols.size(); i++) {
            for (int id = 1; id <= joint_count; id++) {
                if (goal_tols[i].name == joint_names[id - 1]) {
                    id_to_index_2[id - 1] = i;
                }
            }
        }
        // 检查并拷贝数据
        for (int i = 0; i < joint_count; i++) {
            if (id_to_index_2[i] < 0) {
                // 没有给定容差值, 使用默认值
                traj.goal_tol.positions[i] = default_goal_tol.positions[i];
            } else {
                const control_msgs::JointTolerance& tol = goal_tols[id_to_index_2[i]];
                if (tol.position < 0) {
                    // 给定值小于0, 容差值无限制
                    traj.goal_tol.positions[i] = -1;
                } else if (tol.position == 0 || !std::isfinite(tol.position)) {
                    // 给定值为0, 或给定值为异常数, 使用默认容差值
                    traj.goal_tol.positions[i] = default_goal_tol.positions[i];
                } else {
                    // 使用给定容差值
                    traj.goal_tol.positions[i] = tol.position;
                }
            }
        }

        traj.goal_time_tolerance = goal->goal_time_tolerance.toNSec();
    }
};

}
