cmake_minimum_required(VERSION 3.5)
project(octa_ros)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(moveit_ros_planning_interface REQUIRED) 

find_package(rosidl_default_generators REQUIRED) # generate the msg/srv header files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Labviewint.msg"
  "msg/Myint.msg"
  "msg/Intarray.msg"
  "msg/Img.msg"
  "msg/Robotdata.msg"
  "msg/Labviewdata.msg"
 )
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp) # link against interface

# add executables

add_executable(joint_state_listener src/joint_state_listener.cpp)
ament_target_dependencies(joint_state_listener rclcpp std_msgs sensor_msgs)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface std_msgs)
target_link_libraries(test_moveit "${cpp_typesupport_target}")

add_executable(sub_labview_data src/sub_labview_data.cpp)
ament_target_dependencies(sub_labview_data rclcpp std_msgs )
target_link_libraries(sub_labview_data "${cpp_typesupport_target}")

add_executable(pub_robot_data src/pub_robot_data.cpp)
ament_target_dependencies(pub_robot_data rclcpp std_msgs )
target_link_libraries(pub_robot_data "${cpp_typesupport_target}")


# add_executable(sub_int32 src/sub_int32.cpp)
# ament_target_dependencies(sub_int32 rclcpp std_msgs)
# add_executable(pub_int32 src/pub_int32.cpp)
# ament_target_dependencies(pub_int32 rclcpp std_msgs)

# add_executable(sub_int32_ros src/sub_int32_ros.cpp)
# ament_target_dependencies(sub_int32_ros rclcpp std_msgs)

# add_executable(pub_custom_int_labview src/pub_custom_int_labview.cpp)
# ament_target_dependencies(pub_custom_int_labview rclcpp std_msgs)
# target_link_libraries(pub_custom_int_labview "${cpp_typesupport_target}")

# add_executable(add_int src/add_int.cpp)
# ament_target_dependencies(add_int rclcpp std_msgs)

# add_executable(pub_array src/pub_array.cpp)
# ament_target_dependencies(pub_array rclcpp std_msgs )
# target_link_libraries(pub_array "${cpp_typesupport_target}")

# add_executable(sub_array src/sub_array.cpp)
# ament_target_dependencies(sub_array rclcpp std_msgs )
# target_link_libraries(sub_array "${cpp_typesupport_target}")

install(TARGETS
  # pub_array
  # sub_array
  # sub_int32
  # sub_int32_ros
  # pub_custom_int_labview
  # pub_int32
  # add_int
  # pub_custom_int
  # sub_custom_int
  pub_robot_data
  sub_labview_data
  joint_state_listener
  test_moveit
  DESTINATION lib/${PROJECT_NAME})

ament_package()
