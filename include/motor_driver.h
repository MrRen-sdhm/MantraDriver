//
// Created by sdhm on 7/5/19.
//

#ifndef MANTRADRIVER_DRIVER_H
#define MANTRADRIVER_DRIVER_H

#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <array>
#include <memory>

#include "utils.h"
#include "modbusadapter.h"

#define R2D(rad) ((rad) / M_PI * 180)
#define D2R(deg) ((deg) * M_PI / 180)

using namespace std::chrono;

struct MantraDevice {
    static const int cycle_step = 262144; // 编码器一圈分辨率
    array<uint8_t, 7> reduction_ratio = {{120, 120, 120, 100, 100, 100, 100}}; // 齿轮减速比

#pragma pack(push) // 记录当前内存对齐
#pragma pack(1)    // 设定为1字节对齐
    // 内存区间总长度为 (220字节, 110个数据, 存储于110个寄存器) uint16_t -> 2字节 -> 16位 -> 1个数据
    struct Register {
        // 16字节 8个数据
        uint32_t target_position_1; // 40011 offset 0
        uint16_t target_velocity_1;
        uint16_t target_torque_1;
        uint32_t position_value_1;  // 40015 offset 4
        uint16_t velocity_value_1;
        uint16_t torque_value_1;
        // 16字节
        uint32_t target_position_2; // 40019 offset 8
        uint16_t target_velocity_2;
        uint16_t target_torque_2;
        uint32_t position_value_2;  // 40023 offset 12
        uint16_t velocity_value_2;
        uint16_t torque_value_2;
        // 16字节
        uint32_t target_position_3;
        uint16_t target_velocity_3;
        uint16_t target_torque_3;
        uint32_t position_value_3;
        uint16_t velocity_value_3;
        uint16_t torque_value_3;
        // 16字节
        uint32_t target_position_4;
        uint16_t target_velocity_4;
        uint16_t target_torque_4;
        uint32_t position_value_4;
        uint16_t velocity_value_4;
        uint16_t torque_value_4;
        // 16字节
        uint32_t target_position_5;
        uint16_t target_velocity_5;
        uint16_t target_torque_5;
        uint32_t position_value_5;
        uint16_t velocity_value_5;
        uint16_t torque_value_5;
        // 16字节
        uint32_t target_position_6;
        uint16_t target_velocity_6;
        uint16_t target_torque_6;
        uint32_t position_value_6;
        uint16_t velocity_value_6;
        uint16_t torque_value_6;
        // 16字节
        uint32_t target_position_7;
        uint16_t target_velocity_7;
        uint16_t target_torque_7;
        uint32_t position_value_7;
        uint16_t velocity_value_7;
        uint16_t torque_value_7;
        // 14字节
        uint16_t err_code_1;
        uint16_t err_code_2;
        uint16_t err_code_3;
        uint16_t err_code_4;
        uint16_t err_code_5;
        uint16_t err_code_6;
        uint16_t err_code_7;
        // 10字节
        uint16_t motor_status; // 电机状态
        uint32_t back_zero_speed; // 回零速度
        uint32_t jog_speed; // JOG速度
        // 56字节
        uint32_t neg_limit_1; // Joint_1负限位
        uint32_t pos_limit_1; // Joint_1正限位
        uint32_t neg_limit_2; // Joint_2负限位
        uint32_t pos_limit_2; // Joint_2正限位
        uint32_t neg_limit_3; // Joint_3负限位
        uint32_t pos_limit_3; // Joint_3正限位
        uint32_t neg_limit_4; // Joint_4负限位
        uint32_t pos_limit_4; // Joint_4正限位
        uint32_t neg_limit_5; // Joint_5负限位
        uint32_t pos_limit_5; // Joint_5正限位
        uint32_t neg_limit_6; // Joint_6负限位
        uint32_t pos_limit_6; // Joint_6正限位
        uint32_t neg_limit_7; // Joint_7负限位
        uint32_t pos_limit_7; // Joint_7正限位
        // 28字节
        uint32_t joint_speed_1; // Joint_1速度
        uint32_t joint_speed_2; // Joint_2速度
        uint32_t joint_speed_3; // Joint_3速度
        uint32_t joint_speed_4; // Joint_4速度
        uint32_t joint_speed_5; // Joint_5速度
        uint32_t joint_speed_6; // Joint_6速度
        uint32_t joint_speed_7; // Joint_7速度
    } registers;
#pragma pack(pop) // 还原内存对齐

    static uint32_t offset_goal_position(int id) {
//        printf("[DEBUG] offsetOf target_position_1: %zu\n", offsetOf(&Register::target_position_1));
        return offsetOf(&Register::target_position_1) + 8*(id-1);
    }
    static uint32_t offset_curr_position(int id) {
//        printf("[DEBUG] offsetOf position_value_1: %zu\n", offsetOf<>(&Register::position_value_1));
        return offsetOf(&Register::position_value_1) + 8*(id-1);
    }
    static uint32_t offset_curr_velocity(int id) { return offsetOf(&Register::velocity_value_1) + 8*(id-1); }
    static uint32_t offset_curr_effort(int id) { return offsetOf(&Register::torque_value_1) + 8*(id-1); }
//    static uint32_t sizeof_goal_position() { return sizeof(Register::goal_position); }
//
//    static uint32_t offset_read_data() { return offsetOf(&Register::present_position); }
//    static uint32_t sizeof_read_data() { return sizeof(Register::present_position); }

    uint16_t *memory() {
        return (uint16_t *) &registers;
    }
//
//    int32_t _max_position() {
//        static_assert(motor_cycle_step_ % 2 == 0, "motor_cycle_step must divide by 2");
//        return (motor_cycle_step_ / 2) * registers.reduction_ratio - 1;
//    }
//
//    int32_t _min_position() {
//        static_assert(motor_cycle_step_ % 2 == 0, "motor_cycle_step must divide by 2");
//        return -(motor_cycle_step_ / 2) * registers.reduction_ratio;
//    }

//    // 计算电机当前角度, 单位: rad, 有效范围-pi到+pi
//    float present_position(int id) {
//        return float(registers.present_position) / motor_cycle_step_ / registers.reduction_ratio * 2 * float(M_PI);
//    }
//
    // 返回目标位置指针
    uint32_t *goal_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto target_position_1_ptr = (uint16_t *) &registers.target_position_1;
            return (uint32_t *)(target_position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal position!");
        }
    }

    // 返回当前位置指针
    uint32_t *curr_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (uint16_t *)&registers.position_value_1;
            return (uint32_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get current position!");
        }
    }

    // 返回当前速度指针
    uint16_t *curr_vel_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (uint16_t *)&registers.velocity_value_1;
            return (uint16_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal velocity!");
        }
    }

    // 返回当前扭矩指针
    uint16_t *curr_eff_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (uint16_t *)&registers.torque_value_1;
            return (uint16_t *)(position_1_ptr + 8*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("Give a bad id when get goal velocity!");
        }
    }

    // 设定虚拟寄存器中 目标关节角度
    bool set_goal_position(uint8_t id, double rad) {
        if (!std::isfinite(rad)) {
            return false;
        }
        rad = std::min((double) M_PI, rad);
        rad = std::max((double) -M_PI, rad);
        double rounds = rad / (2 * double(M_PI)); // 对应的转数
        auto pos = uint32_t (rounds / reduction_ratio[id] * cycle_step); // 脉冲数 = 转数/减速比*分辨率
//        pos = std::min(_max_position(), pos);
//        pos = std::max(_min_position(), pos);
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
        return uint32_t (*goal_pos_ptr(id));
    }

    // 获取当前位置
    double get_curr_position(uint8_t id) {
        auto pos_raw = uint32_t (*curr_pos_ptr(id)); // 原始位置数据, 单位为度, 并放大10000倍
        double pos = D2R((double)pos_raw / 10000.0f); // 缩小一千倍并转换为弧度
        return pos;
    }
    // 获取当前速度
    double get_curr_velocity(uint8_t id) {
        auto vel_raw = uint32_t (*curr_vel_ptr(id)); // 原始速度数据, 单位为转每分
        double vel = (double)vel_raw * (double)M_PI/30.0f; // 1转每分 = 360/60度每秒 = 2pi/60弧度每秒 = pi/30弧度每秒
        return vel;
    }
    // 获取当前力矩
    double get_curr_effort(uint8_t id) {
        const uint16_t rated_moment = 1; // 额定力矩 FIXME：额定力矩大小
        auto eff_raw = uint32_t (*curr_eff_ptr(id)); // 原始力矩数据, 为额定力矩的千分比
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

    // 总状态
    enum class State {
        WAIT,               // 定时1ms
        WRITE,              // 写入设定位置
        READ,               // 读取关节位置
        SYNC_WRITE_SENDING, // 等待数据发送完毕
        REQUEST,            // 请求读取当前位置
        REQUEST_SENDING,    // 等待请求发送完毕
        WAIT_RESPONSE,      // 等待当前位置响应
        PENDING_RESPONSE,   // 接收响应
    };

    /// 通信状态机相关参数
    // 时序相关参数
    TimePoint last_sync_op_sending_;     // 上一次sync_op开始发送时间
    TimePoint last_sync_op_sent_;     // 上一次sync_op发送完成时间(ns)
    TimePoint last_op_sent_;     // 上一次op发送完成的时间(ns)

    MotorDriver(string ip, int port, int slaver, const string& joint_prefix); // 构造函数
    void spin_once(); // 通信

    // modbus读保持寄存器数据
    int _read_data(int addr, int len, uint16_t* data) {
        int rect = m_master_->modbusReadHoldReg(slaver_, addr, len, data);
        return rect;
    }

    // modbus写保持寄存器数据
    int _write_data(int addr, int len, const uint16_t* data) {
        return m_master_->modbusWriteData(slaver_, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, len, data);
    }

    // 执行读操作
    int _do_read_operation() {
        // TODO：转换为标准单位
        // 读关节位置
        for (int id = 1; id <= motor_cnt_; id++) {
            uint32_t offset = MantraDevice::offset_curr_position(id); // 控制器寄存器位置偏移

            // 从控制器读取当前位置, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
//            printf("[INFO] offset: %d addr: %d\n", offset, hmi_addr_head_ + offset);
            _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(id));
            curr_pos[id-1] = device_.get_curr_position(id); // 当前位置保存到位置缓冲区
        }
        // TODO：读关节速度
        for (int id = 1; id <= motor_cnt_; id++) {
            uint32_t offset = MantraDevice::offset_curr_velocity(id); // 控制器寄存器位置偏移

            // 从控制器读取当前速度, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
            _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_vel_ptr(id));
            curr_vel[id-1] = device_.get_curr_velocity(id); // 当前位置保存到位置缓冲区
        }
        // TODO：读关节力矩
        for (int id = 1; id <= motor_cnt_; id++) {
            uint32_t offset = MantraDevice::offset_curr_effort(id); // 控制器寄存器位置偏移

            // 从控制器读取当前力矩, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
            _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_eff_ptr(id));
            curr_eff[id-1] = device_.get_curr_effort(id); // 当前位置保存到位置缓冲区
        }

//        printf("\nposition_buffer_size:%zu\n", curr_pos.size());
//        for (int i = 0; i < curr_pos.size(); i++) {
//            printf("data: %lu\n", curr_pos[i]);
//        }
        return 1;
    }

    // 执行写操作
    int _do_write_operation() {
        // 写关节位置
        for (int i = 1; i < motor_cnt_; i++) {
            int offset = MantraDevice::offset_goal_position(i); // 控制器寄存器位置偏移
            // 从Mantra寄存器获取目标关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
            _write_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(i)); // 写目标位置到控制器
        }
        return 1;
    }

    // 使能所有关节 写40002的高八位, 第9位控制所有关节
    int enable_all_motor() {
        uint16_t enable_all = 0x0100; // 第9位置1
        _write_data(hmi_addr_power_, 1, &enable_all);
    }

    /// 外部接口
    bool set_position(uint8_t id, double rad) {
        return device_.set_goal_position(id, rad);
    }

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

    volatile State state_ = State::WAIT; // 当前总状态

public:
    std::vector<std::string> joint_names_{}; // 关节名称
    // 关节位置相关参数 TODO：转换为标准单位
    std::array<double, motor_cnt_> curr_pos{}; // 读取位置数据的缓冲区
    std::array<double, motor_cnt_> curr_vel{}; // 读取速度数据的缓冲区
    std::array<double, motor_cnt_> curr_eff{}; // 读取力矩数据的缓冲区
};

}

#endif //MANTRADRIVER_DRIVER_H
