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

#include "blockchain_bots_interfaces/srv/connect_to_sawtooth_sim.hpp"
#include "blockchain_bots_interfaces/srv/send_command_to_sawtooth_sim.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TestServiceClient : public rclcpp::Node {
  private:
    rclcpp::Client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>::SharedPtr client_1;
    rclcpp::Client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>::SharedPtr client_2;
    rclcpp::TimerBase::SharedPtr timer_;
    bool service_done_ = false;
    std::string host_;
    std::int32_t port_, robot_id_;

    void timer_callback1(){
      while (!client_1->wait_for_service(1s)) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
      }
      // Here is the client-specific code which hundles the request
      auto request = std::make_shared<blockchain_bots_interfaces::srv::ConnectToSawtoothSim::Request>();
      request->robot_id = robot_id_;
      request->host = host_;
      request->port = port_;

      service_done_ = false;
      auto result_future = client_1->async_send_request(
        request, std::bind(&TestServiceClient::response_callback1, this,
                           std::placeholders::_1));

    }

    void response_callback1(
      rclcpp::Client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>::SharedFuture future){
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(this->get_logger(), 
                      "Result: success: %i", future.get()->result);
          service_done_ = true;
        } 
        else {
          RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
      }

  public:
    TestServiceClient(std::int32_t robot_id, std::string host, std::int32_t port) : Node("test_service_client")
    {
      robot_id_ = robot_id;
      host_ = host;
      port_ = port;
      client_1 = this->create_client<blockchain_bots_interfaces::srv::ConnectToSawtoothSim>("connect_to_sawtooth");
      client_2 = this->create_client<blockchain_bots_interfaces::srv::SendCommandToSawtoothSim>("send_command_to_sawtooth");
      timer_ = this->create_wall_timer(1s, std::bind(&TestServiceClient::timer_callback1 , this));
    }
    bool is_service_done() const{
      return this->service_done_;
    }



  };


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto service_client1 = std::make_shared<TestServiceClient>(0, "127.0.0.1", 9000);
  while (!service_client1->is_service_done()) {
    rclcpp::spin_some(service_client1);
  }
  auto service_client2 = std::make_shared<TestServiceClient>(1, "127.0.0.1", 9001);
  while (!service_client2->is_service_done()) {
    rclcpp::spin_some(service_client2);
  }  
  rclcpp::shutdown();
  return 0;
}