// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/pid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PID::PID(double dt, double max, double min, double kp, double kd, double ki, double integral_limit)
: integral_limit_(integral_limit),
  dt_(dt),
  max_(max),
  min_(min),
  kp_(kp),
  kd_(kd),
  ki_(ki),
  pre_error_(0),
  integral_(0)
{
  if (
    !std::isfinite(dt) || dt <= 0 || !std::isfinite(max) || !std::isfinite(min) || min > max ||
    !std::isfinite(integral_limit) || integral_limit < 0 || !std::isfinite(kp) ||
    !std::isfinite(kd) || !std::isfinite(ki)) {
    throw std::invalid_argument("Invalid PID configuration");
  }
}

double PID::calculate(double set_point, double pv)
{
  if (!std::isfinite(set_point) || !std::isfinite(pv)) {
    throw std::invalid_argument("Non-finite PID input");
  }
  const double error = set_point - pv;
  integral_ = std::clamp(integral_ + error * dt_, -integral_limit_, integral_limit_);
  const double derivative = first_sample_ ? 0 : (error - pre_error_) / dt_;
  const double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  pre_error_ = error;
  first_sample_ = false;
  return std::clamp(output, min_, max_);
}

void PID::setSumError(double sum_error)
{
  if (!std::isfinite(sum_error)) {
    throw std::invalid_argument("Non-finite PID integral");
  }
  integral_ = std::clamp(sum_error, -integral_limit_, integral_limit_);
}
void PID::reset()
{
  integral_ = pre_error_ = 0;
  first_sample_ = true;
}
PID::~PID() = default;
