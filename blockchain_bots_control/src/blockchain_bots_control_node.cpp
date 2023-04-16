// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <cstdlib>
// #include <future>

// // #include "rclcpp/timer.hpp"
// // #include "rclcpp/logger.hpp"
// // #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "geometry_msgs/msg/pose2_d.hpp"

// #include "blockchain_bots_interfaces/srv/connect_to_sawtooth_sim.hpp"
// #include "blockchain_bots_interfaces/srv/send_command_to_sawtooth_sim.hpp"

#include "blockchain_bots_control/blockchain_bots_control_node.hpp"

using namespace std::chrono_literals;
typedef std::vector<std::vector<std::double_t>> Dictionary;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

namespace blockchain_bots_control
{

PathPlanner::PathPlanner(
  const std::string & node_name, 
  std::int32_t robot_id, 
  std::string host, 
  std::int32_t port) 
: Node(node_name)
{
  connection_to_sawtooth_done_ = false;
  system_ready_for_next_goal_ = false;
  system_ready_for_next_goal_ = false;

  linearSpeedHigh = 0.1;
  lineSpeedLow = 0.01;
  angularSpeedHigh = 0.1;
  angularSpeedLow = 0.01;
  // sawtooth connection
  _robot_id = robot_id;
  host_ = host;
  port_ = port;

  client_1 = this->create_client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>("connect_to_sawtooth");
  client_2 = this->create_client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>("send_command_to_sawtooth");
  
  _connect_to_sawtooth();
  if (robot_id==1){
    _send_command_to_sawtooth(0, "set", 0);
    _send_command_to_sawtooth(1, "set", 0);
    _send_command_to_sawtooth(2, "set", 0);
    _send_command_to_sawtooth(3, "set", 0);
  }

  _robot_name = "robot_" + std::to_string(robot_id);
  std::vector<std::double_t> tmp = {0.5, 0.5, 1.57};
  _goals.push_back(tmp);
  tmp = {0.5, -0.5, 3.14};
  _goals.push_back(tmp);
  tmp = {-0.5, -0.5, 1.57};
  _goals.push_back(tmp);
  tmp = {-0.5, 0.5, 0.0};
  _goals.push_back(tmp);
  _initialize_goals();

  // _kappa_r = 0.5;
  // _kappa_theta = 0.2;
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(_robot_name+"/cmd_vel", 1);
  subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    _robot_name+"/odom_2D", 1, std::bind(&PathPlanner::__odom2D_callback, this, std::placeholders::_1));
}

void PathPlanner::_connect_to_sawtooth(){
  while (!client_1->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
              "Service Unavailable. Waiting for Service...");
  }
  auto request = std::make_shared<blockchain_bots_interfaces::srv::ConnectToSawtoothSim::Request>();
  request->robot_id = _robot_id - 1;
  request->host = host_;
  request->port = port_;

  connection_to_sawtooth_done_ = false;
  auto result_future = client_1->async_send_request(
    request, std::bind(&PathPlanner::response_callback1, this, std::placeholders::_1));

}

void PathPlanner::_send_command_to_sawtooth(std::int32_t station_id, std::string command_type, std::int32_t command_value ){
  while (!client_2->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
              "Service Unavailable. Waiting for Service...");
  }
  auto request = std::make_shared<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim::Request>();
  request->robot_id = _robot_id - 1;
  request->station_num = station_id;
  request->command_type = command_type;
  request->value = command_value;
  
  system_ready_for_next_goal_ = false;

  auto result_future = client_2->async_send_request(
    request, std::bind(&PathPlanner::response_callback2, this, std::placeholders::_1));
}


void PathPlanner::response_callback1(
  rclcpp::Client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>::SharedFuture future){
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), 
                  "Result: success: %i", future.get()->result);
      connection_to_sawtooth_done_ = true;
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

void PathPlanner::response_callback2(
  rclcpp::Client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>::SharedFuture future){
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), 
                  "Result: success: %s", future.get()->result.c_str());
      system_ready_for_next_goal_ = true;
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

std::tuple<std::double_t, std::double_t> PathPlanner::velocity_calculation(std::double_t error_x, std::double_t error_y, std::double_t error_theta)
{
  std::double_t x_vel = 0.0;
  std::double_t angular_vel = 0.0;
  if (_current_goal_idx == 0){
    if (std::abs(error_x) > .02){
      x_vel = lineSpeedLow;  
      if (error_y > 0.25){
        angular_vel = -angularSpeedLow;
      }
      else if (-error_y > 0.25){
        angular_vel = angularSpeedLow;
      }
      else{
        angular_vel = 0.0;
      }
      if (std::abs(error_x)> .05){
        x_vel = linearSpeedHigh;
      }
    }
    else{
      if (std::abs(error_theta)> .05){
        angular_vel = -angularSpeedLow;
        if (std::abs(error_theta)> .314){
          angular_vel = -angularSpeedHigh;
        }
      }
      else{
        if (_robot_id==1){
        _send_command_to_sawtooth(_current_goal_idx,"list",1);
        }
        _send_command_to_sawtooth(_current_goal_idx,"inc",1);
        // while(!is_robot_ready_for_next_goal()){

        // }
        _update_goal_idx();
      }
    }
  } else if (_current_goal_idx == 1){
    if (std::abs(error_y) > .02){
      x_vel = lineSpeedLow;  
      if (error_x > 0.25){
        angular_vel = -angularSpeedLow;
      }
      else if (-error_x > 0.25){
        angular_vel = angularSpeedLow;
      }
      else{
        angular_vel = 0.0;
      }
      if (std::abs(error_y)> .05){
        x_vel = linearSpeedHigh;
      }
    }
    else{
      if (std::abs(error_theta)> .034){
        angular_vel = -angularSpeedLow;
        if (std::abs(error_theta)> .314){
          angular_vel = -angularSpeedHigh;
        }
      }
      else{
        _send_command_to_sawtooth(_current_goal_idx,"inc",1);
        // while(!is_robot_ready_for_next_goal()){
          
        // }
        _update_goal_idx();
      }
    }
  } else if (_current_goal_idx == 2){
    if (std::abs(error_x) > .02){
      x_vel = lineSpeedLow;  
      if (-error_y > 0.25){
        angular_vel = -angularSpeedLow;
      }
      else if (error_y > 0.25){
        angular_vel = angularSpeedLow;
      }
      else{
        angular_vel = 0.0;
      }
      if (std::abs(error_x)> .05){
        x_vel = linearSpeedHigh;
      }
    }
    else{
      if (std::abs(error_theta)> .034){
        angular_vel = -angularSpeedLow;
        if (std::abs(error_theta)> .314){
          angular_vel = -angularSpeedHigh;
        }
      }
      else{
        _send_command_to_sawtooth(_current_goal_idx,"inc",1);
        // while(!is_robot_ready_for_next_goal()){
          
        // }
        _update_goal_idx();
      }
    }
  } else if (_current_goal_idx == 3){
    if (std::abs(error_y) > .02){
      x_vel = lineSpeedLow;  
      if (-error_x > 0.25){
        angular_vel = -angularSpeedLow;
      }
      else if (error_x > 0.25){
        angular_vel = angularSpeedLow;
      }
      else{
        angular_vel = 0.0;
      }
      if (std::abs(error_y)> .05){
        x_vel = linearSpeedHigh;
      }
    }
    else{
      if (std::abs(error_theta)> .05){
        angular_vel = -angularSpeedLow;
        if (std::abs(error_theta)> .314){
          angular_vel = -angularSpeedHigh;
        }
      }
      else{
        // _send_command_to_sawtooth(_current_goal_idx,"show",1);
        _send_command_to_sawtooth(_current_goal_idx,"inc",1);
        // while(!is_robot_ready_for_next_goal()){
          
        // }
        _update_goal_idx();
      }
    }

  }
  
  return std::make_tuple(x_vel, angular_vel);
}


void PathPlanner::_initialize_goals() {
  if (_robot_id == 1){
    _current_goal_idx = 3;
  }
  else{
    _current_goal_idx = 1;
  }
}

void PathPlanner::__odom2D_callback(const geometry_msgs::msg::Pose2D::SharedPtr odom2D_msg)
  {
    auto command_message = geometry_msgs::msg::Twist();
    if (is_connection_service_done()){
    auto state_x = odom2D_msg->x;
    auto state_y = odom2D_msg->y;
    auto state_theta = odom2D_msg->theta;

    auto goal_x = _goals[_current_goal_idx][0];
    auto goal_y = _goals[_current_goal_idx][1];
    auto goal_theta = _goals[_current_goal_idx][2];
    // RCLCPP_INFO(this->get_logger(), "%s, Current GOAL: x='%f', y='%f', theta=%f",_robot_name.c_str(), goal_x, goal_y, goal_theta);

    auto error_x = goal_x - state_x;
    auto error_y = goal_y - state_y;
    auto error_theta = goal_theta - state_theta;
    if (_robot_name=="robot_2" && _current_goal_idx==0){
      error_theta = goal_theta + state_theta;
    }
    // auto command_message = geometry_msgs::msg::Twist();


    std::double_t x_vel, angular_vel;
    std::tie(x_vel, angular_vel) = velocity_calculation(error_x, error_y, error_theta);

    command_message.linear.x = x_vel;
    command_message.angular.z = angular_vel;
    }
    else{
      command_message.linear.x = 0.0;
      command_message.angular.z = 0.0;
    }
    // RCLCPP_INFO(this->get_logger(), "I heard gps point: '%f', '%f'", msg->point.x, msg->point.y);
    publisher_->publish(command_message);
  }

  void PathPlanner::_update_goal_idx(){
    _current_goal_idx = (_current_goal_idx+1) % 4;
  }

    // SAWTOOTH METHODS

    bool PathPlanner::is_connection_service_done() const{
      return this->connection_to_sawtooth_done_;
    }

    bool PathPlanner::is_robot_ready_for_next_goal() const{
      return this->system_ready_for_next_goal_;
    }

}