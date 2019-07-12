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
    cout << endl << "goal_position [";
    for (int i = 1; i <= motor_cnt_; i++) {
        cout << i << ": " << device_.get_goal_position(i) << " ";
    }
    cout << "]" << endl;

    // 获取当前关节角度
    cout << "curr_position [";
    for (int i = 1; i <= motor_cnt_; i++) {
        cout << i << ": " << device_.get_curr_position(i) << " ";
    }
    cout << "]" << endl;
}

void MotorDriver::start() {
    // 通信定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_span_), &MotorDriver::comm_callback, this); // 50HZ
    timer_.start();
}

// 通信回调函数
void MotorDriver::comm_callback(const ros::TimerEvent& e) {

    // 写各关节目标位置
//    do_write_operation();

    // 读各关节当前位置
//    do_read_operation();

    // 获取目标关节角度
//    cout << endl << "goal_position [";
//    for (int i = 1; i <= motor_cnt_; i++) {
//        cout << i << ": " << device_.get_goal_position(i) << " ";
//    }
//    cout << "]" << endl;
//
//    // 获取当前关节角度
//    cout << "curr_position [";
//    for (int i = 1; i <= motor_cnt_; i++) {
//        cout << i << ": " << device_.get_curr_position(i) << " ";
//    }
//    cout << "]" << endl;
}

// 写控制器寄存器
bool MotorDriver::do_write_operation() {
    // 写关节位置
    for (int i = 1; i < motor_cnt_; i++) {
        int offset = MantraDevice::offset_goal_position(i); // 控制器寄存器位置偏移
        // 从Mantra寄存器获取目标关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
        int ret = _write_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(i)); // 写目标位置到控制器
        if (!ret) return false;
    }
    return true;
}

// 读控制器寄存器
bool MotorDriver::do_read_operation() {
    // 读关节位置
    for (int id = 1; id <= motor_cnt_; id++) {
        uint32_t offset = MantraDevice::offset_curr_position(id); // 控制器寄存器位置偏移

        // 从控制器读取当前位置, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
//            printf("[INFO] offset: %d addr: %d\n", offset, hmi_addr_head_ + offset);
        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(id));
        curr_pos[id-1] = device_.get_curr_position(id); // 当前位置保存到位置缓冲区
    }
    // 读关节速度
    for (int id = 1; id <= motor_cnt_; id++) {
        uint32_t offset = MantraDevice::offset_curr_velocity(id); // 控制器寄存器位置偏移

        // 从控制器读取当前速度, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_vel_ptr(id));
        curr_vel[id-1] = device_.get_curr_velocity(id); // 当前位置保存到位置缓冲区
    }
    // 读关节力矩
    for (int id = 1; id <= motor_cnt_; id++) {
        uint32_t offset = MantraDevice::offset_curr_effort(id); // 控制器寄存器位置偏移

        // 从控制器读取当前力矩, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_eff_ptr(id));
        curr_eff[id-1] = device_.get_curr_effort(id); // 当前位置保存到位置缓冲区
    }

//        // FIXME: 调试用, 读取目标位置
//        for (int id = 1; id <= motor_cnt_; id++) {
//            uint32_t offset = MantraDevice::offset_goal_position(id); // 控制器寄存器位置偏移
//            _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(id));
//        }
    return true;
}

}

