//
// Created by sdhm on 7/5/19.
//

#include "motor_driver.h"

namespace Mantra {
    MotorDriver::MotorDriver(string ip, const int port, int slaver) : slaver_(slaver){
        // modbus tcp 初始化
        m_master_ = new ModbusAdapter();
        m_master_->modbusConnectTCP(ip, port);
//        data_buf_ = (uint16_t *) malloc(125 * sizeof(uint16_t));
//        memset(data_buf_, 0, 125 * sizeof(uint16_t));

        // 读取所有控制器寄存器, 并存入MantraDevice Register
        if (!m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, device_.memory())) {
            throw runtime_error("Fail to read all register from controller!");
        }

        // 设置目标关节角度
        device_.set_goal_position(1, M_PI);
        device_.set_goal_position(2, M_PI/2);
        device_.set_goal_position(3, M_PI/4);
        device_.set_goal_position(4, M_PI);
        device_.set_goal_position(5, M_PI/2);
        device_.set_goal_position(6, M_PI/4);
        device_.set_goal_position(7, M_PI/4);

        // 获取目标关节角度
        for (int i = 1; i <= motor_count_; i++) {
            cout << endl << "goal_position" << i << ": " << device_.get_goal_position(i) << endl;
        }

        // 获取当前关节角度
        for (int i = 1; i <= motor_count_; i++) {
            cout << endl << "curr_position: " << i << ": " << device_.get_curr_position(i) << endl;
        }
    }

    MotorDriver::~MotorDriver() {
        delete data_buf_;
    }

// 通信状态机
void MotorDriver::spin_once() {
    TimePoint current_ns = Clock::now();
    cout << "dur_send_sync: " << dur_send_sync_op_ << endl;
    switch (state_) {
        case State::WAIT:
            state_ = State::WRITE;
            break;
        case State::WRITE:
            // 同步写各关节目标位置
//            _generate_sync_operation();

            if (_do_write_operation()) {
                state_ = State::REQUEST;
            } else {
                break;
            }
        case State::READ:
            // 同步写各关节目标位置
            last_sync_op_sending_ = current_ns;
//            _generate_sync_operation();

            if (_do_read_operation()) {
                state_ = State::WAIT;
            } else {
                break;
            }
        default:
            break;
    }
}

}

