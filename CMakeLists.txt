cmake_minimum_required(VERSION 3.10)
project(octa_ros)

set(CMAKE_GENERATOR "Ninja")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 20)
endif()
set(CMAKE_CUDA_STANDARD_REQUIRED OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/W4)
endif()

option(ENABLE_CLANG_TIDY "Run clang-tidy during the build" ON)
option(CLANG_TIDY_WARN_AS_ERRORS "Treat clang-tidy warnings as errors" OFF)
set(_OCTA_CLANG_TIDY_CMD "")
if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES clang-tidy)
  if(CLANG_TIDY_EXE)
    set(_OCTA_CLANG_TIDY_CMD "${CLANG_TIDY_EXE}")
    list(APPEND _OCTA_CLANG_TIDY_CMD "-quiet")
    list(APPEND _OCTA_CLANG_TIDY_CMD "-extra-arg-before=-std=c++${CMAKE_CXX_STANDARD}")

    find_program(_SYS_GXX NAMES ${CMAKE_CXX_COMPILER} g++ c++)
    if(_SYS_GXX)
      execute_process(COMMAND ${_SYS_GXX} -dumpversion
        OUTPUT_VARIABLE _gcc_ver OUTPUT_STRIP_TRAILING_WHITESPACE)
      execute_process(COMMAND ${_SYS_GXX} -print-multiarch
        OUTPUT_VARIABLE _gcc_multiarch OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
      if(_gcc_ver)
        string(REGEX REPLACE "^([0-9]+).*" "\\1" _gcc_major "${_gcc_ver}")
        if(_gcc_major)
          list(APPEND _OCTA_CLANG_TIDY_CMD "-extra-arg-before=-isystem/usr/include/c++/${_gcc_major}")
          if(_gcc_multiarch)
            list(APPEND _OCTA_CLANG_TIDY_CMD "-extra-arg-before=-isystem/usr/include/${_gcc_multiarch}/c++/${_gcc_major}")
          elseif(CMAKE_LIBRARY_ARCHITECTURE)
            list(APPEND _OCTA_CLANG_TIDY_CMD "-extra-arg-before=-isystem/usr/include/${CMAKE_LIBRARY_ARCHITECTURE}/c++/${_gcc_major}")
          endif()
        endif()
      endif()
    endif()

    if(CLANG_TIDY_WARN_AS_ERRORS)
      list(APPEND _OCTA_CLANG_TIDY_CMD "-warnings-as-errors=*")
      message(STATUS "clang-tidy prepared (errors): ${CLANG_TIDY_EXE}")
    else()
      message(STATUS "clang-tidy prepared (warn-only): ${CLANG_TIDY_EXE}")
    endif()
  endif()
endif()

# option(GLIBCXX_USE_CXX11_ABI ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
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
find_package(launch_testing_ament_cmake QUIET)
find_package(onnxruntime REQUIRED CONFIG)

find_library(ONNXRUNTIME_PROVIDERS_CUDA onnxruntime_providers_cuda
  HINTS $ENV{ONNXRUNTIME_LIBDIR}
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/onnxruntime/lib /opt/onnxruntime/lib64)
find_library(ONNXRUNTIME_PROVIDERS_SHARED onnxruntime_providers_shared
  HINTS $ENV{ONNXRUNTIME_LIBDIR}
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/onnxruntime/lib /opt/onnxruntime/lib64)
set(_ONNXRUNTIME_LINK_LIBS ${ONNXRUNTIME_LIBRARY})
if(ONNXRUNTIME_PROVIDERS_SHARED)
  list(APPEND _ONNXRUNTIME_LINK_LIBS ${ONNXRUNTIME_PROVIDERS_SHARED})
endif()
if(ONNXRUNTIME_PROVIDERS_CUDA)
  list(APPEND _ONNXRUNTIME_LINK_LIBS ${ONNXRUNTIME_PROVIDERS_CUDA})
endif()


include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${Open3D_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

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

add_executable(
  coordinator_node src/coordinator_node.cpp src/utils.cpp src/scene_utils.cpp)
ament_target_dependencies(
  coordinator_node
  rclcpp
  rclcpp_action
  action_msgs
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

add_library(octa_motion_utils STATIC src/motion_utils.cpp)
target_include_directories(octa_motion_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(octa_motion_utils geometry_msgs moveit_msgs shape_msgs)

add_executable(focus_node src/focus_node.cpp src/process_img.cpp src/utils.cpp)
ament_target_dependencies(
  focus_node
  rclcpp
  ament_index_cpp
  rclcpp_action
  action_msgs
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
  onnxruntime::onnxruntime
  octa_motion_utils
  Open3D::Open3D
  Eigen3::Eigen)

add_executable(
  move_z_angle_node src/move_z_angle_node.cpp src/utils.cpp)
ament_target_dependencies(
  move_z_angle_node
  rclcpp
  rclcpp_action
  action_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs)
target_link_libraries(
  move_z_angle_node "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}" octa_motion_utils)

add_executable(reset_node src/reset_node.cpp src/utils.cpp)
ament_target_dependencies(
  reset_node
  rclcpp
  rclcpp_action
  action_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  std_msgs)
target_link_libraries(
  reset_node "${cpp_typesupport_target}"
  "${moveit_ros_planning_interface_LIBRARIES}" "${geometry_msgs_LIBRARIES}" octa_motion_utils)

add_executable(freedrive_node src/freedrive_node.cpp)
ament_target_dependencies(
  freedrive_node
  rclcpp
  rclcpp_action
  action_msgs
  moveit_ros_planning moveit_ros_planning_interface
  moveit_msgs shape_msgs geometry_msgs tf2_ros std_srvs
  ur_dashboard_msgs
  controller_manager_msgs
  std_srvs
  std_msgs)
target_link_libraries(freedrive_node "${cpp_typesupport_target}")

add_executable(reconnect_client src/reconnect_node.cpp)
ament_target_dependencies(reconnect_client
  rclcpp rclcpp_action action_msgs
  moveit_ros_planning moveit_ros_planning_interface
  moveit_msgs shape_msgs geometry_msgs tf2_ros std_srvs
  ur_dashboard_msgs std_srvs)
target_link_libraries(reconnect_client "${cpp_typesupport_target}")

add_executable(sub_img src/sub_img.cpp)
ament_target_dependencies(sub_img
  rclcpp rclcpp_action action_msgs
  moveit_ros_planning moveit_ros_planning_interface
  moveit_msgs shape_msgs geometry_msgs tf2_ros std_srvs
  std_msgs OpenCV)
target_link_libraries(sub_img "${cpp_typesupport_target}" "${OpenCV_LIBS}")

add_executable(test_detect src/test_detect.cpp src/process_img.cpp)
ament_target_dependencies(test_detect
  ament_index_cpp OpenCV
  rclcpp rclcpp_action action_msgs
  moveit_ros_planning moveit_ros_planning_interface
  moveit_msgs shape_msgs geometry_msgs tf2_ros std_srvs)
target_link_libraries(test_detect ${OpenCV_LIBS} onnxruntime::onnxruntime "${cpp_typesupport_target}")


add_executable(run_state_listener src/run_state_listener.cpp)
ament_target_dependencies(run_state_listener
  rclcpp rclcpp_action action_msgs
  moveit_ros_planning moveit_ros_planning_interface
  moveit_msgs shape_msgs geometry_msgs tf2_ros std_srvs
  std_msgs)
target_link_libraries(run_state_listener "${cpp_typesupport_target}")


target_compile_definitions(joint_state_publisher PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(focus_node PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(move_z_angle_node PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(reset_node PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(freedrive_node PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(reconnect_client PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(sub_img PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(test_detect PRIVATE OCTA_COORDINATOR_NO_MAIN)
target_compile_definitions(run_state_listener PRIVATE OCTA_COORDINATOR_NO_MAIN)

get_target_property(_ORT_LOCATION onnxruntime::onnxruntime IMPORTED_LOCATION)
if(NOT _ORT_LOCATION)
  get_target_property(_ORT_LOCATION onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)
endif()
if(_ORT_LOCATION)
  get_filename_component(_ORT_LIB_DIR "${_ORT_LOCATION}" DIRECTORY)
  foreach(tgt IN ITEMS focus_node test_detect)
    if(TARGET ${tgt})
      set_target_properties(${tgt} PROPERTIES
        BUILD_RPATH "${_ORT_LIB_DIR}"
        INSTALL_RPATH "${_ORT_LIB_DIR}")
    endif()
  endforeach()
endif()

if(_OCTA_CLANG_TIDY_CMD)
  set(_OCTA_TIDY_TARGETS
      joint_state_publisher
      coordinator_node
      focus_node
      move_z_angle_node
      reset_node
      freedrive_node
      reconnect_client
      sub_img
      test_detect
      run_state_listener)
  foreach(t IN LISTS _OCTA_TIDY_TARGETS)
    if(TARGET ${t})
      set_target_properties(${t} PROPERTIES CXX_CLANG_TIDY "${_OCTA_CLANG_TIDY_CMD}")
    endif()
  endforeach()
  message(STATUS "clang-tidy applied to src/ targets only: ${_OCTA_TIDY_TARGETS}")
endif()

install(
  TARGETS run_state_listener
          sub_img
          joint_state_publisher
          test_detect
          reconnect_client
          coordinator_node
          focus_node
          move_z_angle_node
          reset_node
          freedrive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch urdf srdf bags DESTINATION share/${PROJECT_NAME})

ament_package()

if(launch_testing_ament_cmake_FOUND)
  add_launch_test(test/launch/test_actions.py TARGET test_actions TIMEOUT 60)
  add_launch_test(test/launch/test_bag.py TARGET test_bag TIMEOUT 10800)
  if (DEFINED BUILD_TESTING)
    set_property(TEST test_bag APPEND PROPERTY LABELS fullstack)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(motion_utils_test test/motion_utils_test.cpp)
  if(TARGET motion_utils_test)
    target_link_libraries(motion_utils_test octa_motion_utils)
  endif()
endif()
