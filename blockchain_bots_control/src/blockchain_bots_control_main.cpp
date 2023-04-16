#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "blockchain_bots_control/blockchain_bots_control_node.hpp"

using PathPlanner = blockchain_bots_control::PathPlanner;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto path_planner1 = std::make_shared<PathPlanner>("Path_Planner_node1", 1, "127.0.0.1", 9000);
  auto path_planner2 = std::make_shared<PathPlanner>("Path_Planner_node2", 2, "127.0.0.1", 9001);
  // rclcpp::spin(path_planner1);
  while (true) {
    rclcpp::spin_some(path_planner1);
    rclcpp::spin_some(path_planner2);
  }
  // auto service_client2 = std::make_shared<TestServiceClient>("test_service_client1", 1, "127.0.0.1", 9001);
  // while (!service_client2->is_service_done()) {
  //   rclcpp::spin_some(service_client2);
  // }  

  rclcpp::shutdown();
  return 0;
}