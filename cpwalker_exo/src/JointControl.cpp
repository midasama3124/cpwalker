/**
 * \file JointControl.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */

#include "cpwalker_exo/JointControl.h"

JointControl::JointControl() {
}

JointControl::JointControl(std::string joint, std::vector<bool> hardware) {
  joint_ = joint;
  hardware_ = hardware;
  if (joint_ == "right_knee") {
    //TODO pid_angle_ = PID(0.17,0,0);
  }
  else if (joint_ == "left_knee")
   pid_angle_ = PID(0.17,0,0);
  else if (joint_ == "right_hip"){
    //TODO pid_angle_ = PID(0.17,0,0);
  }
  else if (joint == "left_hip"){
    //TODO pid_angle_ = PID(0.17,0,0);
  }
}

void JointControl::angleCallback(const std_msgs::Float64& msg) {
  angle_ = msg.data;
  std::cout << "Angle message of joint " << joint_ << " of value: " << angle_ << '\n';
}
//Gauge
void JointControl::gaugeCallback(const std_msgs::Float64& msg) {
  gauge_ = msg.data;
  std::cout << "Gauge message of joint " << joint_ << " of value: " << gauge_ << '\n';
}
//FSR1
void JointControl::fsr1Callback(const std_msgs::Float64& msg) {
  //TODO
}
//FSR2
void JointControl::fsr2Callback(const std_msgs::Float64& msg) {
  //TODO
}

JointControl::~JointControl() {

}
