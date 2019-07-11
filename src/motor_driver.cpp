//
// Created by sdhm on 7/5/19.
//

#include "motor_driver.h"

namespace Mantra {
    MotorDriver::MotorDriver(string ip, const int port, int slaver, const string& joint_prefix) : slaver_(slaver){
        // modbus tcp 初始化
        m_master_ = new ModbusAdapter();
        m_master_->modbusConnectTCP(std::move(ip), port);

        // 读取所有控制器寄存器, 并存入MantraDevice Register
        if (!m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, (uint16_t*) device_.memory())) {
            throw runtime_error("Fail to read all register from controller!");
        }

        // 关节名称
        for (int id = 1; id <= motor_cnt_; id++) {
            joint_names_.push_back(joint_prefix + to_string(id));
        }

        enable_all_motor(); // 使能所有关节

        // 设置目标关节角度
//        device_.set_goal_position(1, M_PI);
//        device_.set_goal_position(2, M_PI/2);
//        device_.set_goal_position(3, M_PI/4);
//        device_.set_goal_position(4, -M_PI);
//        device_.set_goal_position(5, -M_PI/2);
//        device_.set_goal_position(6, -M_PI/4);
//        device_.set_goal_position(7, 0);

        // 设置当前关节角度 FIXME：仅用于调试
        device_.set_curr_position(1, -360);
//        device_.set_curr_position(2, 300);
//        device_.set_curr_position(3, 240);
//        device_.set_curr_position(4, 180);
//        device_.set_curr_position(5, 120);
//        device_.set_curr_position(6, 60);
//        device_.set_curr_position(7, 0);

        // 写当前关节角度 FIXME：仅用于调试
        for (int i = 1; i < motor_cnt_; i++) {
            int offset = MantraDevice::offset_curr_position(i); // 控制器寄存器位置偏移
            // 从Mantra寄存器获取当前关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
            _write_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(i)); // 写当前位置到控制器
        }


        // 获取目标关节角度
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << endl << "goal_position" << i << ": " << device_.get_goal_position(i) << endl;
        }

        // 获取当前关节角度
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << endl << "curr_position: " << i << ": " << device_.get_curr_position(i) << endl;
        }
    }

    // 通信状态机
    void MotorDriver::spin_once() {
        TimePoint current_ns = Clock::now();
        switch (state_) {
            case State::WAIT:
                // TODO: 等待系统上电
                state_ = State::WRITE;
                break;
            case State::WRITE:
                // 同步写各关节目标位置
                if (_do_write_operation()) {
                    state_ = State::REQUEST;
                } else {
                    break;
                }
            case State::READ:
                // 同步读各关节当前位置
                if (_do_read_operation()) {
                    state_ = State::WAIT;
                } else {
                    break;
                }
            default:
                break;
        }
        auto dur_spin_once = duration_cast<nanoseconds>(Clock::now() - current_ns).count();
        cout << "dur_spin_once: " << dur_spin_once << endl;

        // 获取目标关节角度
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << endl << "goal_position" << i << ": " << device_.get_goal_position(i) << endl;
        }

        // 获取当前关节角度
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << endl << "curr_position: " << i << ": " << device_.get_curr_position(i) << endl;
        }

}

}

