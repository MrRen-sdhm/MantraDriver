//
// Created by sdhm on 7/5/19.
//

#include "motor_driver.h"

namespace Mantra {
MotorDriver::MotorDriver(string ip, const int port, int slaver, const string& joint_prefix) : slaver_(slaver){
    // modbus tcp 初始化
    m_master_ = new ModbusAdapter();
    m_master_->modbusConnectTCP(std::move(ip), port);

//    set_all_home();
//    enable_all_power(); // 使能所有关节
//    disable_all_power();
//    back_all_home();
//    back_all_home_done();

//    while (true) {
//        // 读取所有控制器寄存器, 并存入MantraDevice Register
//        if (!m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, (uint16_t *) device_.memory())) {
//            throw runtime_error("Fail to read all register from controller!");
//        }
//        if (!m_master_->modbusReadHoldReg(slaver_, hmi_bit_addr_head_, bit_reg_len_, (uint16_t *) device_.bit_memory())) {
//            throw runtime_error("Fail to read all register from controller!");
//        }
//
////        cout << device_.registers.target_position_1 << endl;
//    }

    // 读取所有控制器寄存器, 并存入MantraDevice Register
    if (!m_master_->modbusReadHoldReg(slaver_, hmi_addr_head_, ctrller_reg_len_, (uint16_t *) device_.memory())) {
        throw runtime_error("Fail to read all register from controller!");
    }

    if (!m_master_->modbusReadHoldReg(slaver_, hmi_bit_addr_head_, bit_reg_len_, (uint16_t *) device_.bit_memory())) {
        throw runtime_error("Fail to read all register from controller!");
    }

    // 关节名称
    for (int id = 1; id <= motor_cnt_; id++) {
        joint_names_.push_back(joint_prefix + to_string(id));
    }



//    enable_all_power(); // 使能所有关节
//    disable_all_power();

//    exit(1);

    sub_hmi_ = nh_.subscribe("mantra_hmi", 1, &MotorDriver::hmi_callback, this); // 订阅上位机消息

    // 设置目标关节角度
    device_.set_goal_position(7, -0.01);
//        device_.set_goal_position(1, M_PI);
//        device_.set_goal_position(2, M_PI/2);
//        device_.set_goal_position(3, M_PI/4);
//        device_.set_goal_position(4, -M_PI);
//        device_.set_goal_position(5, -M_PI/2);
//        device_.set_goal_position(6, -M_PI/4);
//        device_.set_goal_position(7, 0);

    // 设置当前关节角度 FIXME：仅用于调试
//        device_.set_curr_position(1, -360);
//        device_.set_curr_position(2, 300);
//        device_.set_curr_position(3, 240);
//        device_.set_curr_position(4, 180);
//        device_.set_curr_position(5, 120);
//        device_.set_curr_position(6, 60);
//        device_.set_curr_position(7, 0);

    // 写当前关节角度 FIXME：仅用于调试
//    for (int i = 1; i < motor_cnt_; i++) {
//        int offset = MantraDevice::offset_curr_position(i); // 控制器寄存器位置偏移
//        // 从Mantra寄存器获取当前关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
//        _write_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(i)); // 写当前位置到控制器
//    }

}

// 写控制器寄存器
bool MotorDriver::do_write_operation() {
    do_write_flag_ = true;
    // 写关节位置
    for (int id = 1; id <= motor_cnt_; id++) {
        int offset = MantraDevice::offset_goal_position(id); // 控制器寄存器位置偏移
        // 从Mantra寄存器获取目标关节角, 写入控制器寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
        int ret = _write_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(id)); // 写目标位置到控制器
        if (!ret) return false;
    }

    if (set_home_flag_ && set_all_home()) { // 发送归零指令
        printf("[INFO] Set current pose as home done.\n");
        set_home_flag_ = false;
    }

    do_write_flag_ = false;
    return true;
}

// 读控制器寄存器
bool MotorDriver::do_read_operation() {
    do_read_flag_ = true;
    // 读关节位置
    for (int id = 1; id <= motor_cnt_; id++) {
        uint32_t offset = MantraDevice::offset_curr_position(id); // 控制器寄存器位置偏移

        // 从控制器读取当前位置, 写入Mantra寄存器, 在控制器寄存器中长度为2单字, 占两个数据位, 即2*uint16_t
//        printf("[INFO] offset[%d]: %d addr: %d\n", id, offset, hmi_addr_head_ + offset);
        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.curr_pos_ptr(id));
        curr_pos[id-1] = device_.get_curr_position(id); // 当前位置保存到位置缓冲区

        // FIXME: 调试用, 将目标位置写入当前位置缓冲区
//        curr_pos[id-1] = device_.get_goal_position(id);
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

    // FIXME: 调试用, 读取目标位置
//    for (int id = 1; id <= motor_cnt_; id++) {
//        uint32_t offset = MantraDevice::offset_goal_position(id); // 控制器寄存器位置偏移
//        _read_data(hmi_addr_head_ + offset, 2, (uint16_t *)device_.goal_pos_ptr(id));
//    }

    print_position();

    do_read_flag_ = false;
    return true;
}

// HMI msg前7位为关节位置(单位rad, 100倍), 第8位为回零标志
void MotorDriver::hmi_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
//    printf("I heard: [ ");
//    for (int i : msg->data) {
//        printf("%d ", i);
//    }
//    printf("]\n");

    // 设置各关节目标位置
    for (int id = 1; id <= motor_cnt_; id++) {
        set_position(id, msg->data[id-1]/100.0);
    }

    // 当前位置设置为初始位置 FIXME:寄存器应该是置位一次即可，还需恢复初始状态应该是置位一次即可，还需恢复初始状态
    if (msg->data[7] == 1) {
        set_home_flag_ = true;
    }
}

void MotorDriver::print_position() {
    if (ros::Time::now().toSec() - last_print_time_ > 1) { // 单位: s
        last_print_time_ = ros::Time::now().toSec();
        // 获取目标关节角度
        cout << endl << "goal_position [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << i << ": " << device_.get_goal_position(i) << " ";
        }
        cout << "] [";
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << i << ": " << R2D(device_.get_goal_position(i)) << " ";
        }
        cout << "]" << endl;

        // 获取当前关节角度
        cout << "curr_position [ ";
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << i << ": " << device_.get_curr_position(i) << " ";
        }
        cout << "] [";
        for (int i = 1; i <= motor_cnt_; i++) {
            cout << i << ": " << R2D(device_.get_curr_position(i)) << " ";
        }
        cout << "]" << endl;
    }
}

}

