/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <iomanip> 

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "My Manual Mode";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName), _node(node)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _actuator_setpoint = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
    _peripheral_actuator_controls = std::make_shared<px4_ros2::PeripheralActuatorControls>(*this);

    // Subscribe to velocity command topic
    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&FlightModeTest::cmdVelCallback, this, std::placeholders::_1));
  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    // Normalize linear velocity to [0, 1]
    float throttle = (_linear_velocity / (2.0f * _max_linear_velocity)) + 0.5f;
    throttle = std::clamp(throttle, 0.0f, 1.0f);

    // Normalize angular velocity to [-1, 1] and scale
    float steering = (_angular_velocity / _max_angular_velocity);
    steering = std::clamp(steering, -1.0f, 1.0f) * 0.8f;

    // Retrieve manual control inputs
    float rc_throttle = _manual_control_input->throttle(); // Forward/Backward throttle
    float rc_steering = _manual_control_input->roll();    // Steering (left/right)

    // Normalize throttle to [0, 1] range
    rc_throttle = (rc_throttle + 1.0f) * 0.5f;
    rc_steering = (rc_steering) * 0.8f;

    if (std::abs(rc_throttle - 0.5f) > std::abs(throttle - 0.5f) ||
          std::abs(rc_steering) > std::abs(steering))
      {
        throttle = rc_throttle;
        steering = rc_steering;
      }

    
    // Fill actuator setpoints
    Eigen::Matrix<float, 12, 1> motor_commands = Eigen::Matrix<float, 12, 1>::Zero();
    motor_commands[0] = throttle;  // Assume motor 0 is primary throttle
    _actuator_setpoint->updateMotors(motor_commands);

    Eigen::Matrix<float, 8, 1> servo_commands = Eigen::Matrix<float, 8, 1>::Zero();
    servo_commands[0] = steering;  // Assume servo 0 controls steering
    _actuator_setpoint->updateServos(servo_commands);

    // Log the command
    RCLCPP_INFO(_node.get_logger(), "Sent command - Throttle: %.2f, Steering: %.2f", throttle, steering);

    // Example to control a servo by passing through RC aux1 channel to 'Peripheral Actuator Set 1'
    _peripheral_actuator_controls->set(_manual_control_input->aux1());
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> _actuator_setpoint;
  std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    _linear_velocity = static_cast<float>(msg->linear.x);
    _angular_velocity = static_cast<float>(msg->angular.z);
  }

  rclcpp::Node & _node;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;

  float _linear_velocity{0.0f};
  float _angular_velocity{0.0f};

  const float _max_linear_velocity{0.5f};    // [m/s], define based on your robot's spec
  const float _max_angular_velocity{1.0f};   // [rad/s], define based on your robot's steering limits
  
};