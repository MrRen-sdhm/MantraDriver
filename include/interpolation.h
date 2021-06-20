//
// Created by sdhm on 13/12/20.
//

#pragma once

#include <utility>
#include <iostream>
#include <vector>
#include <assert.h>
#include <math.h>
#include <time.h>

#include "motor_driver.h"
#include "trajectory.h"

using namespace std;

namespace Mantra {

class LinearInterpolation {
public:
    static const int joint_count_ = MotorDriver::motor_cnt_;

    static pair<double, double> linear(double q0, double q1, double t0, double t1) {
        if(fabs(t0 - t1) < 1e-6) throw runtime_error("t0 and t1 must be different");
        double a0 = q0;
        double a1 = (q1 - q0)/(t1 - t0);
        return {a0, a1};
    }

    void getPosition(const TrajectoryPoint<joint_count_> &begin, const TrajectoryPoint<joint_count_> &end,
                               TrajectoryPoint<joint_count_> &current) {
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            throw runtime_error("The specific time error, time ranges error");
        }

        // unit of time should be second
        double t = current.time_nsec / 1e9;
        double t0 = begin.time_nsec / 1e9;
        double t1 = end.time_nsec / 1e9;

        for (int i = 0; i < joint_count_; i++) {
            double q0 = begin.positions[i];
            double q1 = end.positions[i];

            pair<double, double> ret = linear(q0, q1, t0, t1);
            double a0 = ret.first, a1 = ret.second;
            current.positions[i] = a0 + a1 * (t - t0);
        }
    }
};

class ParabolicInterpolation {
public:
    static const int joint_count_ = MotorDriver::motor_cnt_;

    static vector<double> parabolic(double q0, double q1, double v0, double v1, double t0, double t1, double tf, double qf) {
        if(fabs(t0 - t1) < 1e-6) throw runtime_error("t0 and t1 must be different");
        if((tf <= t0) || (tf >= t1)) throw runtime_error("tf must satisfy t0 < tf < t1");
        if((qf <= min(q0, q1)) || (qf >= max(q0, q1))) throw runtime_error("qf must satisfy min(q0, q1) < qf < max(q0, q1)");

        double T = t1 - t0;
        double h = q1 - q0;
        double Ta = tf - t0;
        double Td = t1 - tf;

        double a0 = q0;
        double a1 = v0;
        double a2 = (2*h - v0*(T + Ta) - v1*Td)/(2*T*Ta);
        double a3 = (2*q1*Ta + Td*(2*q0 + Ta*(v0 - v1)))/(2*T);
        double a4 = (2*h - v0*Ta - v1*Td)/T;
        double a5 = -(2*h - v0*Ta - v1*(T+Td))/(2*T*Td);

        return {a0, a1, a2, a3, a4, a5};
    }

    void getPosition(const TrajectoryPoint<joint_count_> &begin, const TrajectoryPoint<joint_count_> &end,
                               TrajectoryPoint<joint_count_> &current) {
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            throw runtime_error("The specific time error, time ranges error");
        }

        // unit of time should be second
        double t = current.time_nsec / 1e9;
        double t0 = begin.time_nsec / 1e9;
        double t1 = end.time_nsec / 1e9;

        for (int i = 0; i < joint_count_; i++) {
            double q0 = begin.positions[i];
            double q1 = end.positions[i];
            double v0 = begin.velocities[i];
            double v1 = end.velocities[i];

            // symmetric acceleration
            double tf = (t0 + t1)/2;
            double qf = (q0 + q1)/2;

            // asymmetric acceleration, specify tf and qf by users
            // tf = ?
            // qf = ?

            vector<double> ret = parabolic(q0, q1, v0, v1, t0, t1, tf, qf);
            double a0 = ret[0], a1 = ret[1], a2 = ret[2], a3 = ret[3], a4 = ret[4], a5 = ret[5];

            if(t <= tf) {
                current.positions[i] = a0 + a1 * (t - t0) + a2 * pow((t - t0), 2);
            } else {
                current.positions[i] = a3 + a4 * (t - tf) + a5 * pow((t - tf), 2);
            }
        }
    }
};

class CubicInterpolation {
public:
    static const int joint_count_ = MotorDriver::motor_cnt_;

    static vector<double> cubic(double q0, double q1, double v0, double v1, double t0, double t1) {
        if(fabs(t0 - t1) < 1e-6) throw runtime_error("t0 and t1 must be different");

        double T = t1 - t0;
        double h = q1 - q0;

        double a0 = q0;
        double a1 = v0;
        double a2 = (3*h - (2*v0 + v1)*T) / (pow(T, 2));
        double a3 = (-2*h + (v0 + v1)*T) / (pow(T, 3));

        return {a0, a1, a2, a3};
    }

    void getPosition(const TrajectoryPoint<joint_count_> &begin, const TrajectoryPoint<joint_count_> &end,
                            TrajectoryPoint<joint_count_> &current) {
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            throw runtime_error("The specific time error, time ranges error");
        }

        // unit of time should be second
        double t = current.time_nsec / 1e9;
        double t0 = begin.time_nsec / 1e9;
        double t1 = end.time_nsec / 1e9;

        for (int i = 0; i < joint_count_; i++) {
            double q0 = begin.positions[i];
            double q1 = end.positions[i];
            double v0 = begin.velocities[i];
            double v1 = end.velocities[i];

            vector<double> ret = cubic(q0, q1, v0, v1, t0, t1);
            double a0 = ret[0], a1 = ret[1], a2 = ret[2], a3 = ret[3];

            current.positions[i] = a0 + a1*(t - t0) + a2*pow((t - t0), 2) + a3*pow((t - t0), 3); // position
        }
    }
};

class Polynomial5Interpolation {
public:
    static const int joint_count_ = MotorDriver::motor_cnt_;

    static vector<double> polynomial(double q0, double q1, double v0, double v1, double acc0, double acc1, double t0, double t1) {
        if(fabs(t0 - t1) < 1e-6) throw runtime_error("t0 and t1 must be different");

        double T = t1 - t0;
        double h = q1 - q0;

        double a0 = q0;
        double a1 = v0;
        double a2 = acc0/2;
        double a3 = (20*h - (8*v1 + 12*v0)*T - (3*acc0 - acc1)*pow(T, 2)) / (2*pow(T, 3));
        double a4 = (-30*h + (14*v1 + 16*v0)*T + (3*acc0 - 2*acc1)*pow(T, 2)) / (2*pow(T, 4));
        double a5 = (12*h - 6*(v1 + v0)*T + (acc1 - acc0)*pow(T, 2)) / (2*pow(T, 5));

        return {a0, a1, a2, a3, a4, a5};
    }

    void getPosition(const TrajectoryPoint<joint_count_> &begin, const TrajectoryPoint<joint_count_> &end,
                            TrajectoryPoint<joint_count_> &current) {
        if (current.time_nsec < begin.time_nsec || current.time_nsec > end.time_nsec) {
            throw runtime_error("The specific time error, time ranges error");
        }

        // unit of time should be second
        double t = current.time_nsec / 1e9;
        double t0 = begin.time_nsec / 1e9;
        double t1 = end.time_nsec / 1e9;

        for (int i = 0; i < joint_count_; i++) {
            double q0 = begin.positions[i];
            double q1 = end.positions[i];
            double v0 = begin.velocities[i];
            double v1 = end.velocities[i];
            double acc0 = begin.accelerations[i];
            double acc1 = end.accelerations[i];

            vector<double> ret = polynomial(q0, q1, v0, v1, acc0, acc1, t0, t1);
            double a0 = ret[0], a1 = ret[1], a2 = ret[2], a3 = ret[3], a4 = ret[4], a5 = ret[5];

            current.positions[i] = a0 + a1*(t - t0) + a2*pow((t - t0), 2) + a3*pow((t - t0), 3) + a4*pow((t - t0), 4) + a5*pow((t - t0), 5); // position
        }
    }
};

}
