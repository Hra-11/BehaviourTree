cmake_minimum_required(VERSION 3.8)
project(bt_logic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
include_directories(include)
add_executable(bt_logic_node
  src/bt_main.cpp
  src/actions/nav2_client.cpp
)

ament_target_dependencies(bt_logic_node
  rclcpp
  geometry_msgs
  behaviortree_cpp
  rclcpp_action
  nav2_msgs
)

install(TARGETS
  bt_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装行为树 XML 图纸文件夹
install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
