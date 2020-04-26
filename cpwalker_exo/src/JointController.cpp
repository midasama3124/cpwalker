/**
 * \file JointController.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero <p.romero.sorozabal@gmail.com>
 */

#include "cpwalker_exo/JointController.h"

JointController::JointController() {
}

JointController::JointController(std::string joint, std::vector<bool> hardware, int sampling_frequency ) {
  sampling_frequency_ = sampling_frequency;
  joint_ = joint;
  hardware_ = hardware;
  set_point_ = 45;
  controlled_voltage_ = velocity_ = acceleration_ = set_velocity_ = impedance_ = torque_error_ = position_error_ = 0;

  if (joint_ == "right_knee" || joint_ == "left_knee") {
   position_pid_ = PID(0.13, 0 , 0, 9);

   low_impedance_pid_ = PID(0.17,0,1, 9);
   medium_impedance_pid_ = PID(0.5,0,0, 9);
   high_impedance_pid_ = PID(0.1,0,0, 9);

   low_stiffness_ = 0.5;
   low_damping_ = 0.00;
   low_inertia_ = 0;

   medium_stiffness_ = 0.34;
   medium_damping_ = 0.00;
   medium_inertia_ = 0;

   high_stiffness_ = 1.7;
   high_damping_ = 0.00;
   high_inertia_ = 0;
  } else if (joint == "right_hip"){
    //TODO
  }  else if (joint == "left_hip"){
    //TODO
  }
}

void JointController::angleCallback(const std_msgs::Float64& msg) {
  last_angle_ = angle_;
  angle_ = msg.data;
  // Threshold angle variation
  if (last_angle_ != 0) {
    last_velocity_ = velocity_;
    velocity_ = (angle_ - last_angle_) * sampling_frequency_;
  }
  //std::cout << "velocity: " << velocity_ << '\n';
}
//Gauge
void JointController::torqueCallback(const std_msgs::Float64& msg) {
  torque_ = msg.data;
}
//FSR1
void JointController::fsr1Callback(const std_msgs::Float64& msg) {
  //TODO
}
//FSR2
void JointController::fsr2Callback(const std_msgs::Float64& msg) {
  //TODO
}

void JointController::updateSetPoint(float set_point) {
  set_point_ = set_point;
  position_error_ = set_point_ - angle_;
}

bool JointController::setUpJointsControl() {
  //std::cout << "Angle: " << angle_ << '\n';
  if (std::abs(position_error_) > 1) {
    if (angle_ < set_point_){
      controlled_voltage_ = 0.5;
    }
    else if (angle_ > set_point_) {
      controlled_voltage_ = - 0.5;
    }
  } else
    controlled_voltage_ = 0;

  if (controlled_voltage_ == 0)
    return true;
  else
    return false;
}

void JointController::positionControl() {
  controlled_voltage_ = position_pid_.update(position_error_);
}

void JointController::impedanceControl(char impedance_level) {
  switch (impedance_level) {
    case '1':
      impedance_ = impedanceGenerator(low_stiffness_, low_damping_, low_inertia_);
      torque_error_ = impedance_ - torque_;
      controlled_voltage_ = low_impedance_pid_.update(torque_error_);
      break;
    case '2':
      impedance_ = impedanceGenerator(medium_stiffness_, medium_damping_, medium_inertia_);
      torque_error_ = impedance_ - torque_;
      controlled_voltage_ = medium_impedance_pid_.update(torque_error_);
      break;
    case '3':
      impedance_ = impedanceGenerator(high_stiffness_, high_damping_, high_inertia_);
      torque_error_ = impedance_ - torque_;
      controlled_voltage_ = high_impedance_pid_.update(torque_error_);
      break;
  }
  //std::cout << " Level: " << impedance_level << " controlled voltage: " << controlled_voltage_ << '\n';
}

float JointController::impedanceGenerator(float stiffness, float damping, float inertia) {
  // Inertia is not used
  float velocity_error = set_velocity_ - velocity_;
  return position_error_ * stiffness + velocity_error * damping + acceleration_ * inertia;
}

void JointController::mechanicalLimits() {
  // Safety mechanical ranges of motion
  if (joint_ == "right_knee" || joint_ == "left_knee") { // right and left knee limits
    // Upper limit
    if(angle_ >= 68  && controlled_voltage_ > 0) {
      controlled_voltage_ = 0;
      ROS_WARN("The joint [%s] exceeds the upper safety limit of the motion", joint_.c_str());
    }
    // Lower limit
    if (angle_<= 4 && controlled_voltage_ < 0) {
      controlled_voltage_ = 0;
      ROS_WARN("The joint [%s] exceeds the lower safety limit of the motion", joint_.c_str());
    }
  }
  if ( (joint_ == "right_hip" || joint_ == "left_hip") && (angle_ >= 40 || angle_ <= -7) ) { // right and left hip limits
    // Upper limit
    if(angle_ >= 37.3 && controlled_voltage_ > 0) {
      controlled_voltage_ = 0;
      ROS_WARN("The joint [%s] exceeds the upper safety limit of the motion", joint_.c_str());
    }
    // Lower limit
    if (angle_<= -6.2 && controlled_voltage_ < 0) {
      controlled_voltage_ = 0;
      ROS_WARN("The joint [%s] exceeds the lower safety limit of the motion", joint_.c_str());
    }
  }
}


JointController::~JointController() {
}
