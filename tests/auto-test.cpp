#include <asap/tests.hpp>

inline double phi(const double x) {
  double kappa = 5;
  return atan(kappa * log(x));
}

void check_auto_callback(const boost::shared_ptr<const dvs_msgs::EventArray> msg, ros::ServiceClient &client, double &t) {
  double tmin = 1 * 1e-3;
  double tmax = 9 * 1e-3;
  double smin = 1000;
  double smax = 8000;

  double a = (smax - smin) / (phi(tmax) - phi(tmin));
  double b = smax - a * phi(tmax);

  t = saturation(t, tmax, tmin);
  double s = saturation(a * phi(t) + b, smax, smin);

  if(abs(static_cast<double>(msg->events.size()) - s) < 1.5) {
    ROS_INFO("Packet size is %ld and should be %f", msg->events.size(), s);
    t = (std::rand() % 9 + 1) * 1e-3;
  } else {
    ROS_ERROR("Packet size is %ld, but it should be %f", msg->events.size(), s);
  }
  call_feedback_service(client, t);
}

int main(int argc, char **argv) {
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  double t = 5 * 1e-3;

  ros::init(argc, argv, "auto_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<asap::Feedback>("/asap/feedback");
  ros::Subscriber sub = n.subscribe<dvs_msgs::EventArray>("/asap/events", 100, boost::bind(check_auto_callback, _1, boost::ref(client), boost::ref(t)));

  ros::ServiceClient config_client = n.serviceClient<asap::Configuration>("/asap/set_configuration");
  call_configuration_service(config_client, "AUTO", 0);

  ros::spin();

  sub.shutdown();
  client.shutdown();
  config_client.shutdown();
  return 0;
}
