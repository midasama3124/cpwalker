/**
 * \file PID.h
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */


#ifndef CPWALKER_EXO_PID_H_
#define CPWALKER_EXO_PID_H_

#include <iostream>

class PID {
 public:
   PID();
   PID(float kp, float ki, float kd, float limit);

   inline float Proportional() {return kp_;};
   ~PID();

   float update(float error);
   inline float accumulatedError() {return sum_;}

 private:
  float kp_;  //!< Proportional constant
  float ki_;  //!< Integral constant
  float kd_;  //!< Derivative constant
  float sum_;        //!< Sum error
  float last_error_; //!< Last error
  float limit_; //! Limit of the control output desired
  float i_max_, i_min_; //! Boundaries of the Integrator controller.
};

#endif  // CPWALKER_EXO_PID_H_
