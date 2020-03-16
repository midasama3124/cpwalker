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
   PID(double kp, double ki, double kd);
   ~PID();

   double update(double error);

 private:
  double kp_;  //!< Proportional constant
  double ki_;  //!< Integral constant
  double kd_;  //!< Derivative constant
  double sum_;        //!< Sum error
  double last_error_; //!< Last error
};

#endif  // CPWALKER_EXO_PID_H_
