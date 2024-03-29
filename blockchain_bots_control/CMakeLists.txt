cmake_minimum_required(VERSION 3.5)
project(blockchain_bots_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(blockchain_bots_interfaces REQUIRED)

include_directories(include)
# set(library_name ${PROJECT_NAME})
add_library(blockchain_bots_node src/blockchain_bots_control_node.cpp)
ament_target_dependencies(blockchain_bots_node geometry_msgs rclcpp std_msgs blockchain_bots_interfaces)



add_executable(blockchain_bots_control_node src/blockchain_bots_control_main.cpp)
ament_target_dependencies(blockchain_bots_control_node geometry_msgs rclcpp std_msgs blockchain_bots_interfaces)
target_link_libraries(blockchain_bots_control_node 
                    blockchain_bots_node
                     )


add_executable(blockchain_bots_control_test src/blockchain_bots_control_test_node.cpp)
ament_target_dependencies(blockchain_bots_control_test geometry_msgs rclcpp std_msgs blockchain_bots_interfaces)

target_include_directories(blockchain_bots_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(blockchain_bots_control_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  blockchain_bots_control_node
  blockchain_bots_control_test
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
