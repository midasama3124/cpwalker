/**
 * \file PID.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */

#include "cpwalker_exo/PID.h"

PID::PID() {
  
}

PID::PID(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;

  last_error_ = 0;
  sum_ = 0;
}

PID::~PID() {
}

double PID::update(double error) {
  double p, i, d;

  sum_ += error;
  p = kp_ * error;
  i = ki_ * sum_;
  d = kd_ * (error - last_error_);

  last_error_ = error;

  return p + i + d;
}
