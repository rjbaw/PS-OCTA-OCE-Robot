cmake_minimum_required(VERSION 3.5)
project(octa_ros)

set(CMAKE_GENERATOR "Ninja")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Default to C99 if(NOT CMAKE_C_STANDARD) set(CMAKE_C_STANDARD 99) endif()

# # Default to C++23 if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 23)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fmodules-ts)
endif()

# option(GLIBCXX_USE_CXX11_ABI ON) option(BUILD_SHARED_LIBS ON)
# option(BUILD_CUDA_MODULE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# set(msg_files msg/ProgramState.msg msg/RobotMode.msg msg/SafetyMode.msg )

# set(srv_files srv/AddToLog.srv srv/GetLoadedProgram.srv
# srv/GetProgramState.srv srv/GetRobotMode.srv srv/GetSafetyMode.srv
# srv/IsProgramRunning.srv srv/IsProgramSaved.srv srv/Load.srv srv/Popup.srv
# srv/RawRequest.srv )

# set(action_files action/SetMode.action )

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/Labviewint.msg" "msg/Img.msg" "msg/Robotdata.msg"
  "msg/Labviewdata.msg" # ${msg_files} ${srv_files} ${action_files} DEPENDENCIES
                        # action_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

# add executables

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher rclcpp std_msgs sensor_msgs)

add_executable(test_moveit src/test_moveit.cpp src/joint_state_subscriber.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface
                          geometry_msgs tf2_ros std_msgs)

target_link_libraries(
  test_moveit "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}")

add_executable(reset src/reset.cpp src/urscript_publisher.cpp src/utils.cpp)
ament_target_dependencies(reset rclcpp moveit_ros_planning_interface
                          geometry_msgs tf2_ros std_msgs)
target_link_libraries(
  reset "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}")

add_executable(
  octa_ctrl
  src/octa_ctrl.cpp
  src/dds_publisher.cpp
  src/dds_subscriber.cpp
  src/urscript_publisher.cpp
  src/img_subscriber.cpp
  src/process_img.cpp
  src/utils.cpp)
ament_target_dependencies(
  octa_ctrl
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs
  OpenCV
  Open3D
  Eigen3)
target_link_libraries(
  octa_ctrl
  "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}"
  "${geometry_msgs_LIBRARIES}"
  "${OpenCV_LIBS}"
  Open3D::Open3D
  Eigen3::Eigen)

add_executable(get_robot_state src/get_robot_state.cpp)
ament_target_dependencies(get_robot_state rclcpp ur_dashboard_msgs)

# target_include_directories(octa_ctrl PRIVATE
# "/opt/ros/jazzy/include/ur_dashboard_msgs" )
add_executable(reconnect_client src/reconnect_client.cpp src/reconnect_node.cpp)
ament_target_dependencies(reconnect_client rclcpp rclcpp_action
                          ur_dashboard_msgs std_srvs)

add_executable(pub_labview_data src/pub_labview_data.cpp)
ament_target_dependencies(pub_labview_data rclcpp std_msgs)
target_link_libraries(pub_labview_data "${cpp_typesupport_target}")
add_executable(sub_labview_data src/sub_labview_data.cpp)
ament_target_dependencies(sub_labview_data rclcpp std_msgs)
target_link_libraries(sub_labview_data "${cpp_typesupport_target}")

add_executable(pub_robot_data src/pub_robot_data.cpp)
ament_target_dependencies(pub_robot_data rclcpp std_msgs)
target_link_libraries(pub_robot_data "${cpp_typesupport_target}")
add_executable(sub_robot_data src/sub_robot_data.cpp)
ament_target_dependencies(sub_robot_data rclcpp std_msgs)
target_link_libraries(sub_robot_data "${cpp_typesupport_target}")

add_executable(sub_img src/sub_img.cpp)
ament_target_dependencies(sub_img rclcpp std_msgs OpenCV)
target_link_libraries(sub_img "${cpp_typesupport_target}" "${OpenCV_LIBS}")

add_executable(test_img_rot src/test_img_rot.cpp)
ament_target_dependencies(test_img_rot rclcpp std_msgs OpenCV Open3D Eigen3)
target_link_libraries(test_img_rot "${cpp_typesupport_target}" "${OpenCV_LIBS}"
                      Open3D::Open3D Eigen3::Eigen)

add_executable(test_detect src/test_detect.cpp)
ament_target_dependencies(test_detect ament_index_cpp OpenCV)
target_link_libraries(test_detect ${OpenCV_LIBS})

install(
  TARGETS sub_img
          sub_robot_data
          pub_robot_data
          sub_labview_data
          pub_labview_data
          joint_state_publisher
          test_moveit
          test_img_rot
          test_detect
          octa_ctrl
          reset
          get_robot_state
          reconnect_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch urdf srdf DESTINATION share/${PROJECT_NAME})

ament_package()
