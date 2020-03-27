/*
 * \file control_node.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero Sorozábal <p.romero.sorozabal@gmail.com>
 */

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include "cpwalker_exo/JointControl.h"
#include "cpwalker_exo/SPI.h"
#include <bcm2835.h>

//Global lists:
// - Joints in hardware.yaml.
// - Hardware in hardware.yaml.
// - Data processed.
std::string joints_global[4] = {"right_knee", "left_knee", "right_hip", "left_hip"};
std::string hw_names[4] = {"Potentiometer", "Gauge", "FSR1", "FSR2"};
std::string processed_data[4] = {"_angle", "_gauge", "_fsr1", "_fsr2"};
int step_time = 2; //Time of the step in seconds
bool stop_exo = 0; //Panic variable.

// Panic listener function:
void stopMotionExoCallBack (const std_msgs::Bool::ConstPtr& stop) {
  if (stop->data == true) {
    ROS_WARN("Stop exo joints.");
    stop_exo = stop->data;
  }
}

int main(int argc, char **argv) {
  //Initialization of the ROS node.
  ros::init(argc, argv, "exo_control_node");
  ros::NodeHandle nh;

  //// PARAMETERS ////
  //Trajectory list:
  int size_trajectory_vector = 0;
  if (nh.hasParam("trajectory/size"))
    nh.getParam("trajectory/size", size_trajectory_vector);
  else
    ROS_WARN("PARAMETER missing: size of vector in trajectory/");
  std::vector<float> hip_trajectory_vector;   // Hip trajectory vector
  std::vector<float> knee_trajectory_vector;  // Knee trajctory vector
  std::string number, hip_trajectory, knee_trajectory;
  float hip_position, knee_position;
  for (int i = 0; i <= size_trajectory_vector; i++) {
    number = std::to_string(i);
    hip_trajectory = "trajectory/hip_knee_positions/" + std::to_string(i) + "/hip";
    knee_trajectory = "trajectory/hip_knee_positions/" + std::to_string(i) +"/knee";
    if (nh.hasParam("trajectory/hip_knee_positions/" + number)) {
      if (nh.hasParam(hip_trajectory) && nh.hasParam(knee_trajectory)) {
        nh.getParam(hip_trajectory, hip_position);
        hip_trajectory_vector.push_back(hip_position);
        nh.getParam(knee_trajectory, knee_position);
        knee_trajectory_vector.push_back(knee_position);
      } else {
        ROS_WARN("PARAMETER missing: knee/hip joint positions in hip_knee_positions/");
        break;
      }
    }
  }
  //Frequency:
  int frequency;
  if (nh.hasParam("exo_hw/sampling_frequency"))
    nh.getParam("exo_hw/sampling_frequency",frequency);
  else
    ROS_WARN("PARAMETER missing: sampling_frequency in exo_hw/");
  //Available Hardware && Joints lists:
  std::vector<std::vector<bool>> hardware_available;
  std::vector<bool> joints_available;
  hardware_available.resize(4);
  for (int i = 0; i<4; i++) {
    //Check joint.
    if(nh.hasParam("exo_hw/joints/" + joints_global[i])) {
      joints_available.push_back(true);
      //Check joint hardware components.
      for (int j = 0; j<4;j++) {
        bool has_component;
        nh.getParam("exo_hw/joints/" + joints_global[i] + "/" + hw_names[j], has_component);
        hardware_available[i].push_back(has_component);
        std::cout << " " << hardware_available[i][j] << '\n';
      }
    } else {
      joints_available.push_back(false);
      for (int j = 0; j<4;j++) {
        ROS_WARN("PARAMETER missing: hardware components in exo_hw/%s", joints_global[i].c_str());
        hardware_available[i].push_back(false);
        std::cout << " " << hardware_available[i][j] << '\n';
      }
    }
    std::cout << joints_available[i] << '\n';
  }

  //PUBLISHERS
  ros::Publisher right_knee_Vout = nh.advertise<std_msgs::Float64>("/right_knee_Vout", 1000);

  //// SUBSCRIBERS && CONTROL FOR JOINTS ////
  ros::Subscriber stop_motion_exo = nh.subscribe("/stop_motion_exo", 100, &stopMotionExoCallBack);
  JointControl *right_knee_control, *left_knee_control, *right_hip_control, *left_hip_control;
  std::vector<JointControl *> control_joints {right_knee_control, left_knee_control, right_hip_control, left_hip_control};
  for (int i = 0; i<4;i++) {
    for (int j = 0; j<4;j++) {
      if (joints_available[i] == true && hardware_available[i][j] == true) {
        control_joints[i] = new JointControl(joints_global[i], hardware_available[i]);
        if (j==0)
         control_joints[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointControl::angleCallback, control_joints[i]);
        else if (j==1)
          control_joints[i]->getGaugeSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointControl::gaugeCallback, control_joints[i]);
        else if (j==2)
          control_joints[i]->getGaugeSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointControl::gaugeCallback, control_joints[i]);
        else if (j==3)
          control_joints[i]->getGaugeSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointControl::gaugeCallback, control_joints[i]);
      }
    }
  }

  //// SETUP CONTROL LOOP VARIABLES ////
  //Frequency, time and iterator of the trajectory vector.
  ros::Rate loop_rate(frequency);
  int miliseconds = 0;
  int iterator_right_joints = 0;
  int iterator_left_joints = static_cast<int> (size_trajectory_vector / 2);
  //Initialize SPI bus comunication with the joints.
  SPI spi;
  if (!spi.init()){
    ROS_ERROR("SPI ERROR");
    return 1;
  }
  //Send 0V to all joints:
  uint16_t Vout_zero;
  Vout_zero = static_cast<uint16_t > (30215.3); //AD5570 Datasheet equation
  for (int i = 0; i < 4; i++) {
    std::cout << "send 0 volts to joints" << '\n';
    spi.sendData(joints_global[i], Vout_zero);
  }
  //PID Control variables
  float error[4];
  uint16_t controled_data_uint16[4];
  float controled_voltage_float[4];

  char start_therapy;
  std::cout << "Do you want to start the therapy? (s/n)" << '\n';
  std::cin >> start_therapy;

  if (start_therapy == 's'){

    //// CONTROL LOOP ////
    while (ros::ok() && !stop_exo)
    {
      //Time conditions:
      if (miliseconds >= step_time * 1000)
        miliseconds = 0;
      //Every 10ms the consigna changes:
      if (miliseconds%10 == 0) {
        iterator_right_joints = iterator_right_joints + 1;
        iterator_left_joints = iterator_left_joints + 1;
      }
      //Joints position iterators conditions
      if (iterator_right_joints == static_cast<int>(size_trajectory_vector))
        iterator_right_joints = 0;
      if (iterator_left_joints == static_cast<int>(size_trajectory_vector) )
        iterator_left_joints = 0;
      for (int i = 0; i<4;i++) {
        //Check joints availabe.
        if (joints_available[i]) {
          for (int j = 0; j<4;j++) {
            //Check hardware available.
            if (hardware_available[i][j]) {
              std_msgs::Float64 vout_pub; //Vout control of joints to be publish.
              switch (i) {
                //RIGHT KNEE
                case 0:
                    //Angle
                    if (j==0) {
                      error[i] = knee_trajectory_vector[iterator_right_joints] - control_joints[0]->getAngle();
                      error[i] = consigna_angle - control_joints[i]->getAngle();
                      controled_voltage_float[i] = control_joints[i]->getPIDAngle().update(error[i]);
                      controled_data_uint16[i] = static_cast<uint16_t > (controled_voltage_float[i] * 3364.2 + 30215.3); //AD5570 callibration
                      spi.sendData(joints_global[i], controled_data_uint16[i]);
                      vout_pub.data = controled_voltage_float[i];
                      right_knee_Vout.publish(vout_pub);
                      //std::cout << "right_knee: Consigna: " << knee_trajectory_vector[iterator_right_joints] << " | Current angle: " << control_joints[i]->getAngle() << " | Error: " << error[i]  << " | Accumulated error: " << control_joints[0]->getPIDAngle().accumulatedError() << " | Controled signal: " << controled_voltage_float[i] << '\n';
                    }
                    //Gauge
                    else if (j==1) {
                      //TODO
                    }
                    break;
                //LEFT KNEE
                case 1:
                    //Angle
                    if (j==0) {
                      if (controled_voltage_float[i] <= -9)
                          controled_voltage_float[i] = -8.9;
                      if (controled_voltage_float[i] > 10)
                          controled_voltage_float[i] = 10;
                      error[i] = knee_trajectory_vector[iterator_left_joints] - control_joints[0]->getAngle();
                      controled_voltage_float[i] = control_joints[0]->getPIDAngle().update(error[i]);
                      controled_data_uint16[i] = static_cast<uint16_t> (controled_voltage_float[i] * 3364.2 + 30215.3); //AD5570 Datasheet equation
                      spi.sendData(joints_global[i], controled_data_uint16[i]);
                      std::cout << "left_knee: Consigna: " << knee_trajectory_vector[iterator_left_joints] << " | Current angle: " << control_joints[i]->getAngle() << " | Error: " << error[i] << " | Controled signal: " << controled_voltage_float[i] << '\n';
                    }
                    //Gauge
                    else if (j==1) {
                      //TODO
                    }
                    break;
                //RIGHT HIP
                case 2:
                    //Angle
                    if (j==0) {
                      if (controled_voltage_float[i] <= -9)
                          controled_voltage_float[i] = -8.9;
                      if (controled_voltage_float[i] > 10)
                          controled_voltage_float[i] = 10;
                      error[i] = hip_trajectory_vector[iterator_right_joints] - control_joints[0]->getAngle();
                      controled_voltage_float[i] = control_joints[0]->getPIDAngle().update(error[i]);
                      controled_data_uint16[i] = static_cast<uint16_t> (controled_voltage_float[i] * 3364.2 + 30215.3); //AD5570 Datasheet equation
                      spi.sendData(joints_global[i], controled_data_uint16[i]);
                      std::cout << "right_hip: Consigna: " << hip_trajectory_vector[iterator_right_joints] << " | Current angle: " << control_joints[i]->getAngle() << " | Error: " << error[i] << " | Controled signal: " << controled_voltage_float[i] << '\n';
                    }
                    //Gauge
                    else if (j==1) {
                      //TODO
                    }
                    break;
                //LEFT HIP
                case 3:
                    //Angle
                    if (j==0) {
                      if (controled_voltage_float[i] <= -9)
                          controled_voltage_float[i] = -8.9;
                      if (controled_voltage_float[i] > 10)
                          controled_voltage_float[i] = 10;
                      error[i] = hip_trajectory_vector[iterator_left_joints] - control_joints[0]->getAngle();
                      controled_voltage_float[i] = control_joints[0]->getPIDAngle().update(error[i]);
                      controled_data_uint16[i] = static_cast<uint16_t> (controled_voltage_float[i] * 3364.2 + 30215.3); //AD5570 Datasheet equation
                      spi.sendData(joints_global[i], controled_data_uint16[i]);
                      std::cout << "left_hip: Consigna: " << hip_trajectory_vector[iterator_left_joints] << " | Current angle: " << control_joints[i]->getAngle() << " | Error: " << error[i] << " | Controled signal: " << controled_voltage_float[i] << '\n';
                    }
                    //Gauge
                    else if (j==1) {
                      //TODO
                    }
                    break;
              }
            }
          }
        }
      }
      //Count time
      miliseconds = miliseconds + 1;
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  //Put all the SPI outputs to 0V.
  for (int i = 0; i < 4; i++) {
    std::cout << "Stop joints" << '\n';
    spi.sendData(joints_global[i], Vout_zero);
  }
  spi.end();
  control_joints.clear();
  delete right_knee_control;
  delete right_knee_control;
  delete right_hip_control;
  delete right_hip_control;
  return 0;
}
