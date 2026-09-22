#include <asap/tests.hpp>

void check_time_callback(const boost::shared_ptr<const dvs_msgs::EventArray> msg, ros::ServiceClient &client, const double ref) {
  double rate = 1.0 / (msg->events.back().ts.toSec() - msg->events.front().ts.toSec());
  double error = 1 - std::min(rate, ref) / std::max(rate, ref);
  if(error < 0.03) {
    ROS_INFO("Packet rate is %f and should be %f", rate, ref);
  } else {
    ROS_ERROR("Packet rate is %f, but it should be %f", rate, ref);
    call_configuration_service(client, "TIME", static_cast<double>(ref));
  }
}

int main(int argc, char **argv) {
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  double hz = std::rand() % 100 + 5;

  ros::init(argc, argv, "time_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<asap::Configuration>("/asap/set_configuration");
  ros::Subscriber sub = n.subscribe<dvs_msgs::EventArray>("/asap/events", 100, boost::bind(check_time_callback, _1, boost::ref(client), hz));
  ros::spin();
  sub.shutdown();
  client.shutdown();
  return 0;
}
