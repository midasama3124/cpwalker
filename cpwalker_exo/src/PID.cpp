/**
 * \file PID.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */

#include "cpwalker_exo/PID.h"

PID::PID() {

}

PID::PID(float kp, float ki, float kd, float limit) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  limit_ = limit;
  i_max_ = limit_ /ki_;
  i_min_ = -limit_/ki_;

  last_error_ = 0;
  sum_ = 0;
}

PID::~PID() {
}

float PID::update(float error) {
  float p, i, u, d;
  d = kd_ * (error - last_error_);

  //Proportional:
  p = kp_ * error;

  //if (std::abs (error) >= 5)
  //  sum_ = 0;

  // Windup integrator:
  if ( p + ki_ * sum_  <= -limit_) {
    u = -8.9;
    if (error < 0)
      error = 0;
  }
  else if ( p + ki_ * sum_ > limit_) {
    u = 8.9;
    if (error > 0)
      error = 0;
   }

  //Integrator:
  sum_ += error;
  i = ki_ * sum_;

  u = p + i + d;

  last_error_ = error;

  return u;
}
