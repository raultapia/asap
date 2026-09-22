#include <asap/tests.hpp>

void check_size_callback(const boost::shared_ptr<const dvs_msgs::EventArray> msg, ros::ServiceClient &client, const std::size_t ref) {
  if(msg->events.size() == ref) {
    ROS_INFO("Packet size is %ld and should be %ld", msg->events.size(), ref);
  } else {
    ROS_ERROR("Packet size is %ld, but it should be %ld", msg->events.size(), ref);
    call_configuration_service(client, "SIZE", static_cast<double>(ref));
  }
}

int main(int argc, char **argv) {
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  std::size_t size = std::rand() % 9000 + 1000;

  ros::init(argc, argv, "size_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<asap::Configuration>("/asap/set_configuration");
  ros::Subscriber sub = n.subscribe<dvs_msgs::EventArray>("/asap/events", 100, boost::bind(check_size_callback, _1, boost::ref(client), size));
  ros::spin();
  sub.shutdown();
  client.shutdown();
  return 0;
}
