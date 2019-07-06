//
// Created by sdhm on 7/5/19.
//

#include "modbusadapter.h"
#include "motor_driver.h"

int main(int argc, char*argv[]){

    size_t uint8_size = sizeof(uint8_t);
    size_t uint16_size = sizeof(uint16_t);
    size_t uint32_size = sizeof(uint32_t);

    Mantra::MotorDriver *motorDriver = new Mantra::MotorDriver("127.0.0.1", 1502, 1);

    TimePoint current_ns = Clock::now();
    uint32_t dur_time;
    for (;;) {
        dur_time = duration_cast<microseconds>(Clock::now() - current_ns).count();
        if (dur_time > 1000000) {
            cout << "[INFO] time: " << dur_time << endl;
            // 与驱动器通信
            motorDriver->spin_once();
            current_ns = Clock::now();
        }
    }

    return 0;
}

