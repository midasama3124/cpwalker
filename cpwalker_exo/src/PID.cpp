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
  float p, i, d;

  //Proportional:
  p = kp_ * error;
  //Integrator:
  sum_ += error;
  //Integrator windup: 
  if (sum_ >= i_max_)
    sum_ = i_max_;
  else if (sum_ <= i_min_)
    sum_ = i_min_;
  else
    i = ki_ * sum_;
  // Diferential:
  d = kd_ * (error - last_error_);

  last_error_ = error;

  if ( p + i + d <= -9)
      return -8.9;
  else if ( p + i + d > 9)
      return 8.9;
  else
    return p + i + d;
}
