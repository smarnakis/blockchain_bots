#ifndef BLOCKCHAIN_BOTS_CONTROL__CONTROL__NODE_HPP_
#define BLOCKCHAIN_BOTS_CONTROL__CONTROL__NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <future>

#include "rclcpp/timer.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "blockchain_bots_interfaces/srv/connect_to_sawtooth_sim.hpp"
#include "blockchain_bots_interfaces/srv/send_command_to_sawtooth_sim.hpp"

using namespace std::chrono_literals;
typedef std::vector<std::vector<std::double_t>> Dictionary;

namespace blockchain_bots_control
{

class PathPlanner : public rclcpp::Node {
  public:
    explicit PathPlanner(
        const std::string & node_name, 
        std::int32_t robot_id, 
        std::string host, 
        std::int32_t port);
    ~PathPlanner() override = default;

    void _initialize_goals();

    void __odom2D_callback(
        const geometry_msgs::msg::Pose2D::SharedPtr odom2D_msg);

    void _update_goal_idx();

    bool is_connection_service_done() const;
    bool is_robot_ready_for_next_goal() const;
  
  private:
    std::string host_;
    std::int32_t port_, _robot_id, _current_goal_idx;
    std::string _robot_name;
    Dictionary _goals;
    std::float_t linearSpeedHigh, lineSpeedLow, angularSpeedHigh, angularSpeedLow;


    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscriber_;

    // SAWTOOTH
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>::SharedPtr client_1;
    rclcpp::Client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>::SharedPtr client_2;
    bool connection_to_sawtooth_done_;
    bool system_ready_for_next_goal_;
    // bool system_ready_for_next_goal_ = false;


    void _connect_to_sawtooth();

    void _send_command_to_sawtooth(
        std::int32_t station_id, 
        std::string command_type, 
        std::int32_t command_value);

    void response_callback1(
      rclcpp::Client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>::SharedFuture future);

    void response_callback2(
      rclcpp::Client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>::SharedFuture future);

    std::tuple<std::double_t, std::double_t> velocity_calculation(
        std::double_t error_x, 
        std::double_t error_y, 
        std::double_t error_theta);
};

}

#endif // BLOCKCHAIN_BOTS_CONTROL__CONTROL__NODE_HPP_