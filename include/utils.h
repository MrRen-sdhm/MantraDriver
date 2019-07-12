//
// Created by sdhm on 7/5/19.
//

#ifndef MANTRADRIVER_UTILS_H
#define MANTRADRIVER_UTILS_H

// 获取寄存器偏移量
template<typename T, typename U>
constexpr size_t offsetOf(U T::*member) {
    return (int16_t *) &((T *) nullptr->*member) - (int16_t *) nullptr; }

#endif //MANTRADRIVER_UTILS_H
