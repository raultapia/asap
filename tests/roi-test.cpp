#include <asap/tests.hpp>

void check_dvs_roi_callback(const boost::shared_ptr<const dvs_msgs::EventArray> msg, ros::ServiceClient &client, const cv::Rect &roi) {
  int cnt = 0;
  for(const auto e : msg->events) {
    if(!cv::Point(e.x, e.y).inside(roi)) {
      cnt++;
    }
  }
  if(!cnt) {
    ROS_INFO("All the events in the packet are inside (%d,%d)[%dx%d]", roi.x, roi.y, roi.width, roi.height);
  } else {
    ROS_ERROR("%d out of %ld events are outside (%d,%d)[%dx%d]", cnt, msg->events.size(), roi.x, roi.y, roi.width, roi.height);
    call_roi_service(client, roi);
  }
}

void check_aps_roi_callback(const boost::shared_ptr<const sensor_msgs::Image> msg, ros::ServiceClient &client, const cv::Rect &roi) {
  if(static_cast<int>(msg->width) == roi.width && static_cast<int>(msg->height) == roi.height) {
    ROS_INFO("Frame from APS is of size [%dx%d] and should be of size [%dx%d]", msg->width, msg->height, roi.width, roi.height);
  } else {
    ROS_ERROR("Frame from APS is of size [%dx%d] and should be of size [%dx%d]", msg->width, msg->height, roi.width, roi.height);
    call_roi_service(client, roi);
  }
}

int main(int argc, char **argv) {
  cv::Rect roi(30, 30, 100, 100);
  ros::init(argc, argv, "roi_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<asap::Roi>("/asap/set_roi");
  ros::Subscriber sub_dvs = n.subscribe<dvs_msgs::EventArray>("/asap/events", 100, boost::bind(check_dvs_roi_callback, _1, boost::ref(client), boost::ref(roi)));
  ros::Subscriber sub_aps = n.subscribe<sensor_msgs::Image>("/asap/image_raw", 100, boost::bind(check_aps_roi_callback, _1, boost::ref(client), boost::ref(roi)));
  ros::spin();
  sub_dvs.shutdown();
  sub_aps.shutdown();
  client.shutdown();
  return 0;
}
