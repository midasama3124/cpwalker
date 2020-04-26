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
#include <array>
#include <vector>
#include "cpwalker_exo/JointController.h"
#include "cpwalker_exo/SPI.h"
#include <bcm2835.h>
#include <math.h>
#include <fstream>

//Global lists:
std::string joints_global[4]  = {"right_knee", "left_knee", "right_hip", "left_hip"};
std::string hw_names[4]       = {"Potentiometer", "Gauge", "FSR1", "FSR2"};
std::string processed_data[4] = {"_angle", "_gauge", "_fsr1", "_fsr2"};

//Global user variables
int step_time = 2; //Time of the step in seconds
int steps_number = 4;

//Panic variable.
bool stop_exo = 0;

//Panic listener function:
void stopMotionExoCallBack (const std_msgs::Bool::ConstPtr& stop) {
  if (stop->data == true) {
    stop_exo = stop->data;
    ROS_WARN("Stop exo joints.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "exo_control_node");
  ros::NodeHandle nh;


  int size_trajectory_vector = 0;
  int sampling_frequency;                             //sampling_frequency of the control
  std::vector<float> hip_trajectory_vector;           // Hip trajectory vector
  std::vector<float> knee_trajectory_vector;          // Knee trajctory vector
  std::vector<bool> joints_available;                 // List of availabe joints
  std::vector<std::vector<bool>> hardware_available;  // List of availabe hardware per joint
  hardware_available.resize(4);                       // Hardware list for each joint

  // Get hip-knee trajectory vector size from trajectory.yaml file
  if (nh.hasParam("trajectory/size"))
    nh.getParam("trajectory/size", size_trajectory_vector);
  else
    ROS_ERROR("PARAMETER missing: size of vector in trajectory/");

  //Get control frequency from hardware.yaml file
  if (nh.hasParam("exo_hw/sampling_frequency"))
    nh.getParam("exo_hw/sampling_frequency",sampling_frequency);
  else
    ROS_ERROR("PARAMETER missing: sampling_frequency in exo_hw/");

  // Check joints and hardware available in the hardware.yaml file
  for (int i = 0; i<4; i++) {
    if(nh.hasParam("exo_hw/joints/" + joints_global[i])) {//Joint.
      joints_available.push_back(true);
      for (int j = 0; j<4;j++) {
        bool has_component;
        nh.getParam("exo_hw/joints/" + joints_global[i] + "/" + hw_names[j], has_component);
        hardware_available[i].push_back(has_component);
      }
    } else {
      joints_available.push_back(false);
      for (int j = 0; j<4;j++) {
        ROS_WARN("PARAMETER missing: hardware components in exo_hw/%s", joints_global[i].c_str());
        hardware_available[i].push_back(false);
      }
    }
  }

  // Get hip and knee trajectories in the trajectory.yaml file
  for (int i = 0; i <= size_trajectory_vector; i++) {
    if (nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i))) {
      if (nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i) + "/hip") && nh.hasParam("trajectory/hip_knee_positions/" + std::to_string(i) +"/knee")) {
        float position;
        nh.getParam("trajectory/hip_knee_positions/" + std::to_string(i) + "/hip", position);
        hip_trajectory_vector.push_back(position);
        nh.getParam("trajectory/hip_knee_positions/" + std::to_string(i) +"/knee", position);
        knee_trajectory_vector.push_back(position);
      } else {
        ROS_ERROR("PARAMETER missing: Joint positions in hip_knee_positions/");
        break;
      }
    } else {
      ROS_ERROR("PARAMETER missing: Joint positions in hip_knee_positions/");
      break;
    }
  }

  // Publish controlled voltage applied to the right knee
  ros::Publisher right_knee_Vout = nh.advertise<std_msgs::Float64>("/right_knee_Vout", 1000);

  // Subscribe the "JointController" of each joint to the angle and gauge topics
  //JointController *right_knee_control, *left_knee_control, *right_hip_control, *left_hip_control;
  std::vector<JointController *> joint_controller(4);
  for (int i = 0; i<4;i++) {
    joint_controller[i] = new JointController(joints_global[i], hardware_available[i], sampling_frequency);
    if (joints_available[i]) {
      for (int j = 0; j<4;j++) {
       if (hardware_available[i][j]) {
        if (j==0)
         joint_controller[i]->getAngleSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::angleCallback, joint_controller[i]);
        else if (j==1)
          joint_controller[i]->getTorqueSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::torqueCallback, joint_controller[i]);
        else if (j==2)
          joint_controller[i]->getTorqueSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::torqueCallback, joint_controller[i]);
        else if (j==3)
          joint_controller[i]->getTorqueSubscriber() = nh.subscribe("/" + joints_global[i] + processed_data[j],100, &JointController::torqueCallback, joint_controller[i]);
        }
      }
    }
  }

  // Subscribe to the panic topic
  ros::Subscriber stop_motion_exo = nh.subscribe("/stop_motion_exo", 100, &stopMotionExoCallBack);

  // SPI communication
  SPI spi;
  if (!spi.init()){
    ROS_ERROR("SPI ERROR");
    return 1;
  }
  // Stop all joints.
  float Vout_zero = 0;
  for (int i = 0; i < 4; i++) {
    spi.sendData(joints_global[i], Vout_zero);
  }

  // Type of therapy
  char therapy_type = 's';
  char impedance_level;
  char start_gait;
  bool starting_point = true;
  // Start conditions
  bool ready[4] = { false, false, false, false };
  char prepare;
  bool continue_therapy = true;

  // Control loop variables
  ros::Rate loop_rate(sampling_frequency);
  int time = 0;
  int iterator_right_joints = 0;
  int iterator_left_joints = static_cast<int> (size_trajectory_vector / 2); // Defase the position of the left joints
  std_msgs::Float64 vout_pub;

  // Save therapy info.
  const char *path="/home/pi/Documents/Test.data";
  std::ofstream testFile(path);

  for (int i = 0; i < 4; i++) {
    if(joints_available[i] && !hardware_available[i][0])
      ROS_ERROR(" The position sensor of the %s has not been detected, it is not possible to perform any therapy without this sensor.",joints_global[i].c_str());
  }

  // CONTROL LOOP
  while (ros::ok() && continue_therapy && !stop_exo ) {
    // Time conditions:
    if (time >= step_time * sampling_frequency * steps_number) {
      time = 0;
      therapy_type = 's';

      //Close file with the info of the therapy.
      testFile.close();

      //Put initial condidions and stop the joint motors.
      starting_point = true;
      for (int i = 0; i < 4; i++) {
        spi.sendData(joints_global[i], Vout_zero);
        ready[i] = false;
        joint_controller[i]->setControlledVoltage(Vout_zero);
      }

    }

    // Change set points depending on the duration of the step:
    if (time % (step_time * sampling_frequency / size_trajectory_vector) == 0) {
      iterator_right_joints = iterator_right_joints + 1;
      iterator_left_joints = iterator_left_joints + 1;
    }

    // Loop conditions for joint position iterators
    if (iterator_right_joints == static_cast<int>(size_trajectory_vector))
      iterator_right_joints = 0;
    if (iterator_left_joints == static_cast<int>(size_trajectory_vector) )
      iterator_left_joints = 0;

    // Update set points of the Joints:
    for (int i = 0; i<4;i++) {
      if (joints_available[i]) {
        switch (i) {
          case 0: // RIGHT KNEE
            //joint_controller[i]->updateSetPoint(knee_trajectory_vector[iterator_right_joints]);
            joint_controller[i]->updateSetPoint(45);
            break;
          case 1: // LEFT KNEE
            joint_controller[i]->updateSetPoint(knee_trajectory_vector[iterator_left_joints]);
            break;
          case 2: // RIGHT HIP
            joint_controller[i]->updateSetPoint(hip_trajectory_vector[iterator_right_joints]);
            break;
          case 3: // LEFT HIP
            joint_controller[i]->updateSetPoint(hip_trajectory_vector[iterator_left_joints]);
            break;
        };
      }
    }

    // Control movement depending on therapy type
    switch (therapy_type) {
      case 's':
        // Prerare joints, move joints to their start positions
        time = 0;
        iterator_right_joints = 0;
        iterator_left_joints = static_cast<int> (size_trajectory_vector / 2);

        // Starting point
        for (int i = 0; i < 4; i++) {
          if(ready[i] || joint_controller[i]->getControlledVoltage() != 0)
            starting_point = false;
        }
        if (starting_point) {
          do {
            std::cout << "**** Do you want to start the therapy? ****" << '\n' << "*Important information: " << '\n';
            for (int i = 0; i<4;i++) {
              if (!joints_available[i]) {
                std::cout << " " << joints_global[i] << " is NOT detected." << '\n';
              } else {
                std::cout << " " << joints_global[i] << " is detected." << '\n';
                for (int j = 0; j<4;j++) {
                  if (!hardware_available[i][j])
                    std::cout << "  -" << hw_names[j] << " is not detected." << '\n';
                }
              }
            }
            std::cout  << '\n' << "Answer (y: yes | n: no/exit ): ";
            std::cin >> prepare;
            if (prepare == 'y'){
              continue_therapy = true;
              therapy_type = 's'; // 's' : Set up joints position, move them to initial positios
              std::cout << '\n' << "Moving joints to their start positions..." << '\n' << '\n' << '\n';
            } else {
              continue_therapy = false;
            }
          } while(prepare != 'y' && prepare != 'n');
        }

        // If joints are in their start positions ask for the type of therapy.
        if (ready[0] && ready[1] && ready[2] && ready[3]) {
          do {
            std::cout  << '\n' << "**** What type of therapy do you desire? ****" << '\n' << '\n' << "Answer ( p: position control | i: impedance control ): ";
            std::cin >> therapy_type;
          } while ( therapy_type != 'p' && therapy_type != 'i');

          if (therapy_type == 'i') {
            do {
              std::cout  << '\n' << "**** What impedance intensity do you want? ****" << '\n' << '\n' << "Answer ( 1: low | 2: medium | 3: high ): ";
              std::cin >> impedance_level;
            } while( impedance_level != '1' && impedance_level != '2' && impedance_level != '3' && impedance_level != 'e');
          }

          do {
            std::cout << '\n' << "**** How many steps do you want to perform? ****" << '\n' << '\n' << "Answer: ";
            std::cin.clear(); // clear the error flags
            std::cin.ignore(INT_MAX, '\n'); // discard the row
          } while (!(std::cin >> steps_number));

          do {
            std::cout << '\n' << "**** Start gait? (y,n) ****" << '\n' << '\n' << "Answer (y: yes | n: no ): ";
            std::cin >> start_gait;
            // Reset to initial conditions
            if (start_gait == 'n') {
              therapy_type = 's';
              for (int i = 0; i < 4; i++) {
                starting_point = true;
                ready[i] = false;
                joint_controller[i]->setControlledVoltage(Vout_zero);
              }
            } else {
              if(!testFile.is_open())
                std::cout << "ERROR no info can be saved in the file." << '\n';
              std::cout << '\n' << "Moving joints ..." << '\n' << '\n' << '\n';
            }
          } while(start_gait != 'y' && start_gait != 'n');
        } else {
          // Check what jointsare not in their start positions
          for (int i = 0; i<4;i++) {
            if (joints_available[i]) {
              if (hardware_available[i][0]) {
                if (!ready[i])
                  ready[i] = joint_controller[i]->setUpJointsControl();
              }
            } else
              ready[i] = true;
          }
        }
        break;
      case 'p':
        // Position control of the joints
        for (int i = 0; i<4;i++) {
          // Trajectory control
          if (joints_available[i])
            joint_controller[i]->positionControl();
        }
        break;
      case 'i':
        // Impedance control of the joints
        for (int i = 0; i<4;i++) {
          if (joints_available[i]) {
            if (!hardware_available[i][1]) {
              ROS_ERROR("The torque sensor has not been detected, the impedance control cannot be accomplished without the position sensor.");
            } else {
              // Impedance control
              joint_controller[i]->impedanceControl(impedance_level);
            }
          }
        }
        break;
    }

    // File information
    std::string file_info = std::to_string(time) + " " + std::to_string(joint_controller[0]->getSetPosition()) + " " + std::to_string(joint_controller[0]->getAngle()) + '\n';
    testFile << file_info;

    // Move joints
    for (int i = 0; i<4;i++) {
      if (joints_available[i]) {

        // Check mechanical limits
        joint_controller[i]->mechanicalLimits();

        // Send data
        spi.sendData(joint_controller[i]->getJoinName(), joint_controller[i]->getControlledVoltage());
        vout_pub.data = joint_controller[i]->getControlledVoltage();
        right_knee_Vout.publish(vout_pub);
      }
    }

    //Count time
    time = time + 1;
    ros::spinOnce();
    loop_rate.sleep();
  }

  //Put all the SPI outputs to 0V.
  for (int i = 0; i < 4; i++) {
    spi.sendData(joints_global[i], Vout_zero);
  }
  spi.end();
  joint_controller.clear();

  return 0;
}
