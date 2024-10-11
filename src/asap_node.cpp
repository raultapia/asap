#include "asap/asap.hpp"

#if ROS == 1
#include "ros/ros.h"
#elif ROS == 2
#include "rclcpp/rclcpp.hpp"
#endif

int main(int argc, char **argv) {
#if ROS == 1
  /*** --- ROS setup --- ***/
  ros::init(argc, argv, "asap");

  /*** --- Start publishing --- ***/
  Asap asap(std::make_shared<ros::NodeHandle>("/asap/ros__parameters"));
  asap.run();
  ros::shutdown();
#elif ROS == 2
  /*** --- ROS setup --- ***/
  rclcpp::init(argc, argv);

  /*** --- Start publishing --- ***/
  Asap asap(rclcpp::Node::make_shared("asap"));
  asap.run();
  rclcpp::shutdown();
#endif

  /*** --- That's all folks --- ***/
  return 0;
}
