/*
 * \file control_node.cpp
 *
 * \date Created on march 10, 2020
 * \author Pablo Romero Sorozábal <p.romero.sorozabal@gmail.com>
 */

#ifndef CPWALKER_EXO_JOINTCONTROL_H_
#define CPWALKER_EXO_JOINTCONTROL_H_

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "cpwalker_exo/PID.h"

class JointControl {
public:

  JointControl();
  JointControl(std::string joint, std::vector<bool> hardware);

  //Return a pointer to the subscriber of the joint.
  inline ros::Subscriber& getAngleSubscriber() {return angle_subscriber_;}
  inline ros::Subscriber& getGaugeSubscriber() {return gauge_subscriber_;}
  inline ros::Subscriber& getFsr1Subscriber() {return fsr1_subscriber_;}
  inline ros::Subscriber& getFsr2Subscriber() {return fsr2_subscriber_;}

  inline PID& getPIDAngle() {return pid_angle_;}

  //Return values of the private variables.
  inline float getAngle() const {return angle_;}
  inline float getGauge() const {return gauge_;}
  inline float getFsr1() const {return fsr1_;}
  inline float getFsr2() const {return fsr2_;}

  //Angle
  void angleCallback(const std_msgs::Float64& msg);
  //Gauge
  void gaugeCallback(const std_msgs::Float64& msg);
  //FSR1
  void fsr1Callback(const std_msgs::Float64& msg);
  //FSR2
  void fsr2Callback(const std_msgs::Float64& msg);

  ~JointControl();

private:
  std::string joint_;     //!< Name of the joint to be control.
  std::vector<bool> hardware_;  //!< List of the hardware componets available in the joint.
  ros::Subscriber angle_subscriber_;
  ros::Subscriber gauge_subscriber_;
  ros::Subscriber fsr1_subscriber_;
  ros::Subscriber fsr2_subscriber_;
  float angle_;  //!< Postition of the potentiometer.
  float gauge_;     //!< Value of the gauge.
  float fsr1_;   //!< Value of the fsr1.
  float fsr2_;  //!< Value of the fsr2.
  PID pid_angle_;
};

#endif  // CPWALKER_EXO_JOINTCONTROL_H_
