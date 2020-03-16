/*
 * \file control_node.cpp
 *
 * \date Created on feb 26, 2020
 * \author Pablo Romero Sorozábal <p.romero.sorozabal@gmail.com>
 */

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include "cpwalker_exo/JointControl.h"
#include <bcm2835.h>

//Global lists:
// - Joints in hardware.yaml.
// - Hardware in hardware.yaml.
// - Data processed.
std::string joints_global[4] = {"right_knee", "left_knee", "right_hip", "left_hip"};
std::string hw_names[4] = {"Potentiometer", "Gauge", "FSR1", "FSR2"};
std::string processed_data[4] = {"_angle", "_gauge", "_fsr1", "_fsr2"};

int main(int argc, char **argv) {
  //Initialization of the ROS node.
  ros::init(argc, argv, "exo_control_node");
  ros::NodeHandle nh;

  ////PARAMETERS////
  //TRAJECTORY LIST
  int size_trajectory_vector = 0;
  if (nh.hasParam("trajectory/size"))
    nh.getParam("trajectory/size", size_trajectory_vector);
  else
    ROS_WARN("PARAMETER missing: size of vector in trajectory/");

  std::vector<float> hip_trajectory_vector;   // Hip trajectory vector:
  std::vector<float> knee_trajectory_vector;  // Knee trajctory vector:
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
  //FREQUENCY:
  int frequency;
  if (nh.hasParam("exo_hw/sampling_frequency"))
    nh.getParam("exo_hw/sampling_frequency",frequency);
  else
    ROS_WARN("PARAMETER missing: sampling_frequency in exo_hw/");
  //HARDWARE LIST
  std::vector<std::vector<bool>> hardware;
  hardware.resize(4);
  for (int i = 0; i<4; i++) {
    //Check joint.
    if(nh.hasParam("exo_hw/joints/" + joints_global[i])) {
      //Check joint hardware components.
      for (int j = 0; j<4;j++) {
        bool has_component;
        nh.getParam("exo_hw/joints/" + joints_global[i] + "/" + hw_names[j], has_component);
        hardware[i].push_back(has_component);
      }
    } else {
      for (int j = 0; j<4;j++) {
        ROS_WARN("PARAMETER missing: hardware components in exo_hw/%s", joints_global[i].c_str());
        hardware[i].push_back(false);
      }
    }
  }

  ////CREATE CONTROL FOR JOINTS AND SUBSCRIPTION TO TOPICS////
  JointControl *right_knee_control, *left_knee_control, *right_hip_control, *left_hip_control;
  std::vector<JointControl *> control_joints {right_knee_control, left_knee_control, right_hip_control, left_hip_control};
  for (int i = 0; i<4;i++) {
    for (int j = 0; j<4;j++) {
      if (hardware[i][j] == true) {
        control_joints[i] = new JointControl(joints_global[i], hardware[i]);
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

  ////CONTROL LOOP////
  ros::Rate loop_rate(frequency);
  //Initialize time and iterator of the trajectory vector.
  int miliseconds = 0;
  int iterator;

/*
  ////SETUP SPI COMMUNICATIONS////
  if (!bcm2835_init()) {
    printf("bcm2835_init failed. Are you running as root??\n");
    return 1;
  }
  // Here are selected the GPIO's that are going to be used as Chanel Select (4 in this case)
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_29, BCM2835_GPIO_FSEL_OUTP);//right_knee
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_31, BCM2835_GPIO_FSEL_OUTP);//left_knee
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_36, BCM2835_GPIO_FSEL_OUTP);//right_hip
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_37, BCM2835_GPIO_FSEL_OUTP);//left_hip
  //Unsellect channels.
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29, HIGH);
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_31, HIGH);
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_36, HIGH);
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_37, HIGH);
  bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_33, HIGH);
  //Start SPI operations.  Forces RPi AUX SPI pins P1-36 (MOSI), P1-38 (MISO), P1-40 (CLK) and P1-36 (CE2) to alternate function ALT4, which enables those pins for SPI interface.
  if (!bcm2835_aux_spi_begin()) {
   printf("bcm2835_spi_begin failed. Are you running as root??\n");
   return 1;
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // In AD5570: "Data is t$
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // CPOL = 0, CPHA = 1,  $
  //bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); // RPi4 = 2.083 MHz ,$
  bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32); //RPi4= 8.333MHz
  //4 slaves implementation:
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  */

  float error_left_knee;
  unsigned int controled_left_knee_signal;
  while (ros::ok())
  {
    //Begining and end of step:
    if (miliseconds == 0 || miliseconds == 2000) {
      miliseconds = 0;
      iterator = 0;
    }

    //Every 10ms de consigna changes:
    if (miliseconds%10 == 0)
      iterator = iterator + 1;

    //Control de output:
    // Right_knee
    error_left_knee  = knee_trajectory_vector[iterator] - control_joints[1]->getAngle();
    controled_left_knee_signal = static_cast<unsigned int>(control_joints[1]->getPIDAngle().update(error_left_knee));
    std::cout << "Consigna: " << knee_trajectory_vector[iterator] << " | Current angle: " << control_joints[1]->getAngle() << " | Controled signal: " << controled_left_knee_signal << '\n';

/*
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29,LOW); //right_knee
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_31,HIGH);  //left_knee
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_36,HIGH); //right_hip
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_37,HIGH); //left_hip
    bcm2835_aux_spi_transfern(controled_left_knee_signal, sizeof(controled_left_knee_signal));

    //Unsellect channels.
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_29, HIGH);
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_31, HIGH);
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_36, HIGH);
    bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_37, HIGH);

    // Let the signal of the chanels select high for at least 45ns, AD5570 datasheet.
    bcm2835_st_delay(0, 0.5);
*/
    //Count time
    miliseconds = miliseconds + 1;
    ros::spinOnce();
    loop_rate.sleep();
  }

/*
  //END AUX SPI.
  bcm2835_aux_spi_end();
  //Close the library, deallocating any allocated memory and closing /dev/mem
  if (!bcm2835_close())
    ROS_WARN("ERROR. It was not possible to close the library bcm2835.");
*/

  //delete pid_left_knee_angle;
  control_joints.clear();
  delete right_knee_control;
  delete left_knee_control;
  delete right_hip_control;
  delete left_hip_control;
  return 0;
}
