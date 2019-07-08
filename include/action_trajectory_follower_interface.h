//
// Created by sdhm on 7/7/19.
//

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

#include "motor_driver.h"

struct TrajectoryPoint
{
  std::array<double, Mantra::MotorDriver::motor_cnt_> positions;
  std::array<double, Mantra::MotorDriver::motor_cnt_> velocities;
  std::chrono::microseconds time_from_start;

  TrajectoryPoint()
  {
  }

  TrajectoryPoint(std::array<double, Mantra::MotorDriver::motor_count_> &pos,
          std::array<double, Mantra::MotorDriver::motor_count_> &vel, std::chrono::microseconds tfs)
    : positions(pos), velocities(vel), time_from_start(tfs)
  {
  }
};

class ActionTrajectoryFollowerInterface
{
public:
  virtual bool start() = 0;
  virtual bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt) = 0;
  virtual void stop() = 0;
  virtual ~ActionTrajectoryFollowerInterface(){};
};
