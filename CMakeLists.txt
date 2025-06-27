cmake_minimum_required(VERSION 3.10)
project(octa_ros)

set(CMAKE_GENERATOR "Ninja")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# option(GLIBCXX_USE_CXX11_ABI ON) option(BUILD_CUDA_MODULE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(msg_files "msg/Labviewint.msg" "msg/Img.msg" "msg/Robotdata.msg"
              "msg/Labviewdata.msg")

set(srv_files srv/Scan3d.srv)

set(action_files
    action/Focus.action action/Freedrive.action action/MoveZAngle.action
    action/Reset.action action/FullScan.action)

set(ROSIDL_DEPS builtin_interfaces action_msgs unique_identifier_msgs)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
                           ${action_files} DEPENDENCIES ${ROSIDL_DEPS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

# add executables

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher rclcpp std_msgs sensor_msgs)

add_executable(coordinator_node src/coordinator_node.cpp src/utils.cpp)
ament_target_dependencies(
  coordinator_node
  rclcpp
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs
  OpenCV
  Open3D
  Eigen3)
target_link_libraries(
  coordinator_node
  "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}"
  "${geometry_msgs_LIBRARIES}"
  "${OpenCV_LIBS}"
  Open3D::Open3D
  Eigen3::Eigen)

add_executable(focus_node src/focus_node.cpp src/process_img.cpp src/utils.cpp)
ament_target_dependencies(
  focus_node
  rclcpp
  rclcpp_action
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs
  OpenCV
  Open3D
  Eigen3)
target_link_libraries(
  focus_node
  "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}"
  "${geometry_msgs_LIBRARIES}"
  "${OpenCV_LIBS}"
  Open3D::Open3D
  Eigen3::Eigen)

add_executable(move_z_angle_node src/move_z_angle_node.cpp src/utils.cpp)
ament_target_dependencies(
  move_z_angle_node
  rclcpp
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs)
target_link_libraries(
  move_z_angle_node "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}")

add_executable(reset_node src/reset_node.cpp src/utils.cpp)
ament_target_dependencies(
  reset_node
  rclcpp
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs)
target_link_libraries(
  reset_node "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}")

add_executable(freedrive_node src/freedrive_node.cpp)
ament_target_dependencies(
  freedrive_node
  rclcpp
  rclcpp_action
  ur_dashboard_msgs
  controller_manager_msgs
  std_srvs
  std_msgs)
target_link_libraries(freedrive_node "${cpp_typesupport_target}")

# target_include_directories(octa_ctrl PRIVATE
# "/opt/ros/jazzy/include/ur_dashboard_msgs" )
add_executable(reconnect_client src/reconnect_node.cpp)
ament_target_dependencies(reconnect_client rclcpp rclcpp_action
                          ur_dashboard_msgs std_srvs)

add_executable(sub_img src/sub_img.cpp)
ament_target_dependencies(sub_img rclcpp std_msgs OpenCV)
target_link_libraries(sub_img "${cpp_typesupport_target}" "${OpenCV_LIBS}")

add_executable(test_detect src/test_detect.cpp src/process_img.cpp)
ament_target_dependencies(test_detect ament_index_cpp OpenCV)
target_link_libraries(test_detect ${OpenCV_LIBS})

install(
  TARGETS sub_img
          joint_state_publisher
          test_detect
          reconnect_client
          coordinator_node
          focus_node
          move_z_angle_node
          reset_node
          freedrive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch urdf srdf DESTINATION share/${PROJECT_NAME})

ament_package()
