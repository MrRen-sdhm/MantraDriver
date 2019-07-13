//
// Created by sdhm on 7/5/19.
//

#pragma once

#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <utility>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <array>
#include <memory>

#include <ros/ros.h>

#include "utils.h"
#include "modbusadapter.h"

#define R2D(rad) ((rad) / M_PI * 180)
#define D2R(deg) ((deg) * M_PI / 180)

using namespace std::chrono;

struct MantraDevice {
    static const int cycle_step = 262144; // 编码器一圈分辨率
    array<uint8_t, 7> reduction_ratio = {{120, 120, 120, 100, 100, 100, 100}}; // 齿轮减速比

#pragma pack(push) // 记录当前内存对齐
#pragma pack(1)    // 设定为1字节对齐, 高地址对应高位, 如4011对应低八位, 4012对应高八位
    // 内存区间总长度为 (220字节, 110个数据, 存储于110个寄存器) uint16_t -> 2字节 -> 16位 -> 1个数据
    struct Register {
        // 16字节 8个数据
        int32_t target_position_1; // 40011-40012 offset 0
        int16_t target_velocity_1;
        int16_t target_torque_1;
        int32_t position_value_1;  // 40015-40016 offset 4
        int16_t velocity_value_1;
        int16_t torque_value_1;
        // 16字节
        int32_t target_position_2; // 40019-40020 offset 8
        int16_t target_velocity_2;
        int16_t target_torque_2;
        int32_t position_value_2;  // 40023-40025 offset 12
        int16_t velocity_value_2;
        int16_t torque_value_2;
        // 16字节
        int32_t target_position_3;
        int16_t target_velocity_3;
        int16_t target_torque_3;
        int32_t position_value_3;
        int16_t velocity_value_3;
        int16_t torque_value_3;
        // 16字节
        int32_t target_position_4;
        int16_t target_velocity_4;
        int16_t target_torque_4;
        int32_t position_value_4;
        int16_t velocity_value_4;
        int16_t torque_value_4;
        // 16字节
        int32_t target_position_5;
        int16_t target_velocity_5;
        int16_t target_torque_5;
        int32_t position_value_5;
        int16_t velocity_value_5;
        int16_t torque_value_5;
        // 16字节
        int32_t target_position_6;
        int16_t target_velocity_6;
        int16_t target_torque_6;
        int32_t position_value_6;
        int16_t velocity_value_6;
        int16_t torque_value_6;
        // 16字节
        int32_t target_position_7;
        int16_t target_velocity_7;
        int16_t target_torque_7;
        int32_t position_value_7;
        int16_t velocity_value_7;
        int16_t torque_value_7;
        // 14字节
        int16_t err_code_1;
        int16_t err_code_2;
        int16_t err_code_3;
        int16_t err_code_4;
        int16_t err_code_5;
        int16_t err_code_6;
        int16_t err_code_7;
        // 10字节
        int16_t motor_status; // 电机状态
        int32_t back_zero_speed; // 回零速度
        int32_t jog_speed; // JOG速度
        // 56字节
        int32_t neg_limit_1; // Joint_1负限位
        int32_t pos_limit_1; // Joint_1正限位
        int32_t neg_limit_2; // Joint_2负限位
        int32_t pos_limit_2; // Joint_2正限位
        int32_t neg_limit_3; // Joint_3负限位
        int32_t pos_limit_3; // Joint_3正限位
        int32_t neg_limit_4; // Joint_4负限位
        int32_t pos_limit_4; // Joint_4正限位
        int32_t neg_limit_5; // Joint_5负限位
        int32_t pos_limit_5; // Joint_5正限位
        int32_t neg_limit_6; // Joint_6负限位
        int32_t pos_limit_6; // Joint_6正限位
        int32_t neg_limit_7; // Joint_7负限位
        int32_t pos_limit_7; // Joint_7正限位
        // 28字节
        int32_t joint_speed_1; // Joint_1速度
        int32_t joint_speed_2; // Joint_2速度
        int32_t joint_speed_3; // Joint_3速度
        int32_t joint_speed_4; // Joint_4速度
        int32_t joint_speed_5; // Joint_5速度
        int32_t joint_speed_6; // Joint_6速度
        int32_t joint_speed_7; // Joint_7速度
    } registers;
#pragma pack(pop) // 还原内存对齐

    static uint8_t offset_goal_position(int id) {
//        printf("[DEBUG] offsetOf target_position_1: %zu\n", offsetOf(&Register::target_position_1));
        return offsetOf(&Register::target_position_1) + 8*(id-1);
    }
    static uint8_t offset_curr_position(int id) {
//        printf("[DEBUG] offsetOf position_value_1: %zu\n", offsetOf<>(&Register::position_value_1));
        return offsetOf(&Register::position_value_1) + 8*(id-1);
    }
    static uint8_t offset_curr_velocity(int id) { return offsetOf(&Register::velocity_value_1) + 8*(id-1); }
    static uint8_t offset_curr_effort(int id) { return offsetOf(&Register::torque_value_1) + 8*(id-1); }
//    static uint32_t sizeof_goal_position() { return sizeof(Register::goal_position); }
//
//    static uint32_t offset_read_data() { return offsetOf(&Register::present_position); }
//    static uint32_t sizeof_read_data() { return sizeof(Register::present_position); }

    int16_t *memory() {
        return (int16_t *) &registers;
    }
//
//    int32_t _max_position(uint8_t id) {
//        static_assert(cycle_step % 2 == 0, "motor_cycle_step must divide by 2");
//        return (cycle_step / 2) * reduction_ratio[id] - 1;
//    }
//
//    int32_t _min_position(uint8_t id) {
//        static_assert(cycle_step % 2 == 0, "motor_cycle_step must divide by 2");
//        return -(cycle_step / 2) * reduction_ratio[id];
//    }

//    // 计算电机当前角度, 单位: rad, 有效范围-pi到+pi
//    float present_position(int id) {
//        return float(registers.present_position) / motor_cycle_step_ / registers.reduction_ratio * 2 * float(M_PI);
//    }
//
    // 返回目标位置指针
    int32_t *goal_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto target_position_1_ptr = (int16_t *) &registers.target_position_1;
            return (int32_t *)(target_position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal position!");
        }
    }

    // 返回当前位置指针
    int32_t *curr_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.position_value_1;
            return (int32_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get current position!");
        }
    }

    // 返回当前速度指针
    int16_t *curr_vel_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.velocity_value_1;
            return (int16_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal velocity!");
        }
    }

    // 返回当前扭矩指针
    int16_t *curr_eff_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.torque_value_1;
            return (int16_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal velocity!");
        }
    }

    // 设定虚拟寄存器中 目标关节角度
    bool set_goal_position(uint8_t id, double rad) {
        if (!std::isfinite(rad)) {
            return false;
        }
//        rad = std::min((double) M_PI, rad);
//        rad = std::max((double) -M_PI, rad);
        double rounds = rad / (2 * double(M_PI)); // 对应的转数
        auto pos = int32_t (rounds / reduction_ratio[id-1] * cycle_step); // 脉冲数 = 转数/减速比*分辨率
        // FIXME:限位
//        pos = std::min(_max_position(id), pos);
//        pos = std::max(_min_position(id), pos);
        *goal_pos_ptr(id) = pos; // 写位置

        return true;
    }

    // 设定虚拟寄存器中 目标关节角度 FIXME:仅用于调试
    bool set_curr_position(uint8_t id, double deg) {
        auto pos = uint32_t (deg * 10000);
        *curr_pos_ptr(id) = pos; // 写位置
        return true;
    }

    // 获取目标位置
    double get_goal_position(uint8_t id) {
        auto cycle = int32_t(*goal_pos_ptr(id));
        double rounds = (double)(cycle) / cycle_step * reduction_ratio[id-1];
        return rounds * (2 * double(M_PI));
    }

    // 获取当前位置
    double get_curr_position(uint8_t id) {
        auto pos_raw = int32_t (*curr_pos_ptr(id)); // 原始位置数据, 单位为度, 并放大10000倍
        double pos = D2R((double)pos_raw / 10000.0f); // 缩小一千倍并转换为弧度
        return pos;
    }
    // 获取当前速度
    double get_curr_velocity(uint8_t id) {
        auto vel_raw = int32_t (*curr_vel_ptr(id)); // 原始速度数据, 单位为转每分
        double vel = (double)vel_raw * (double)M_PI/30.0f; // 1转每分 = 360/60度每秒 = 2pi/60弧度每秒 = pi/30弧度每秒
        return vel;
    }
    // 获取当前力矩
    double get_curr_effort(uint8_t id) {
        const uint16_t rated_moment = 1; // 额定力矩 FIXME：额定力矩大小
        auto eff_raw = int32_t (*curr_eff_ptr(id)); // 原始力矩数据, 为额定力矩的千分比
        double eff = (double)eff_raw/1000.0f * rated_moment;
        return eff;
    }
};

namespace Mantra {
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

class MotorDriver {
public:
    static const uint8_t motor_cnt_ = 7; // 电机数
    std::vector<std::string> joint_names_{}; // 关节名称

    /// 关节位置相关参数
    std::array<float, motor_cnt_> curr_pos{}; // 读取位置数据的缓冲区
    std::array<float, motor_cnt_> curr_vel{}; // 读取速度数据的缓冲区
    std::array<float, motor_cnt_> curr_eff{}; // 读取力矩数据的缓冲区

    MotorDriver(string ip, int port, int slaver, const string& joint_prefix); // 构造函数

    /// 外部接口
    // 设置虚拟寄存器中关节位置
    bool set_position(uint8_t id, double rad) {
        return device_.set_goal_position(id, rad);
    }
    bool do_write_operation(); // 写控制器寄存器
    bool do_read_operation();  // 读控制器寄存器

private:
    /// modbus相关参数
    ModbusAdapter *m_master_; // modbus服务器
    int slaver_; // modbus客户端id

    /// 寄存器相关参数
    MantraDevice device_; // 机械臂
    const uint32_t hmi_addr_head_ = 40011; // HMI字节操作首地址
    const uint32_t hmi_addr_power_ = 40002; // 高八位为关节使能地址, 10-16位控制各关节使能, 9位控制所有关节
    size_t reg_size_ = sizeof(typename MantraDevice::Register); // MantraDevice::Register尺寸
    size_t ctrller_reg_len_ = reg_size_/2; // 控制器寄存器数量, 即uint16_t数据量

    // modbus读保持寄存器数据, 使用uint16_t类型写入虚拟寄存器, 使用数据时将虚拟寄存器数据转换为int16或int32类型即可
    int _read_data(int addr, int len, uint16_t* data) {
        int rect = m_master_->modbusReadHoldReg(slaver_, addr, len, data);
        return rect;
    }

    // modbus写保持寄存器数据, 使用uint16_t类型写入, int16或int32类型数据需转换为uint16类型
    int _write_data(int addr, int len, const uint16_t* data) {
        return m_master_->modbusWriteData(slaver_, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, len, data);
    }

    // 使能所有关节 写40002的高八位, 第9位控制所有关节
    int enable_all_motor() {
        int16_t enable_all = 0x0100; // 第9位置1
        _write_data(hmi_addr_power_, 1, (uint16_t*) &enable_all);
    }
};

}

