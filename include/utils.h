//
// Created by sdhm on 7/5/19.
//

#ifndef MANTRADRIVER_UTILS_H
#define MANTRADRIVER_UTILS_H


template<typename T, typename U>
constexpr size_t offsetOf(U T::*member) {
    return (uint16_t *) &((T *) nullptr->*member) - (uint16_t *) nullptr;
}


#endif //MANTRADRIVER_UTILS_H
