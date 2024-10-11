cmake_minimum_required(VERSION 3.10.0)
project(asap)

if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 1)
  set(ROS1 TRUE)
elseif(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
  set(ROS2 TRUE)
endif()

find_package(OpenEV REQUIRED)
if(ROS1)
  find_package(catkin REQUIRED COMPONENTS cv_bridge dvs_msgs dynamic_reconfigure message_generation roscpp std_msgs)
elseif(ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(dvs_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
endif()

include_directories(include)
include_directories(${OpenEV_INCLUDE_DIRS})
if(ROS1)
  include_directories(${catkin_INCLUDE_DIRS})
elseif(ROS2)
  include_directories(${cv_bridge_INCLUDE_DIRS})
  include_directories(${dvs_msgs_INCLUDE_DIRS})
  include_directories(${rclcpp_INCLUDE_DIRS})
  include_directories(${sensor_msgs_INCLUDE_DIRS})
  include_directories(${std_msgs_INCLUDE_DIRS})
endif()

if(ROS1)
  add_service_files(FILES Feedback.srv)
  add_service_files(FILES Configuration.srv)
  add_service_files(FILES Roi.srv)
  generate_messages(DEPENDENCIES sensor_msgs std_msgs)
  generate_dynamic_reconfigure_options(config/parameters.cfg)
  catkin_package(CATKIN_DEPENDS dynamic_reconfigure message_runtime sensor_msgs std_msgs)
elseif(ROS2)
  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "srv/Feedback.srv"
    "srv/Configuration.srv"
    "srv/Roi.srv"
    DEPENDENCIES
    sensor_msgs
    std_msgs)
endif()

add_executable(asap_node src/asap_node.cpp src/configuration.cpp src/main_thread.cpp src/publishers.cpp src/services.cpp)
if(ROS1)
  add_dependencies(asap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_compile_options(asap_node PRIVATE "-DROS=1")
  target_link_libraries(asap_node ${catkin_LIBRARIES})
  target_link_libraries(asap_node ${OpenEV_LIBRARIES})
elseif(ROS2)
  target_compile_options(asap_node PRIVATE "-DROS=2")
  ament_target_dependencies(
    asap_node
    cv_bridge
    dvs_msgs
    OpenEV
    rclcpp
    sensor_msgs
    std_msgs)
  target_link_libraries(asap_node ${OpenEV_LIBRARIES})
  install(TARGETS asap_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY ${OpenEV_LIBRARY_DIRS}/ DESTINATION lib)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  rosidl_target_interfaces(asap_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
  ament_package()
endif()
