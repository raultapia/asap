#include "asap/asap.hpp"

int main(int argc, char **argv) {
  /*** --- ROS setup --- ***/
  ros::init(argc, argv, "asap");
  ros::NodeHandle n;

  /*** --- Start publishing --- ***/
  Asap asap(n);
  asap.run();

  /*** --- That's all folks --- ***/
  return 0;
}
