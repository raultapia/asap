#ifndef ASAP_TEST_HPP
#define ASAP_TEST_HPP

#include <asap/Configuration.h>
#include <asap/Feedback.h>
#include <asap/Roi.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

inline void call_configuration_service(ros::ServiceClient &client, const std::string &mode, const double param) {
  asap::Configuration srv;
  srv.request.aps_enabled = true;
  srv.request.aps_exposure = 6500;
  srv.request.aps_rate = 40;
  srv.request.dvs_enabled = true;
  srv.request.dvs_gamma = 1;
  srv.request.dvs_mode = mode;
  srv.request.dvs_rate = (mode == "TIME") ? param : 0;
  srv.request.dvs_size = (mode == "SIZE") ? param : 0;
  srv.request.imu_enabled = true;
  client.call(srv);
}

inline void call_roi_service(ros::ServiceClient &client, const cv::Rect &roi) {
  asap::Roi srv;
  srv.request.roi.x_offset = roi.x;
  srv.request.roi.y_offset = roi.y;
  srv.request.roi.width = roi.width;
  srv.request.roi.height = roi.height;
  client.call(srv);
}

inline void call_feedback_service(ros::ServiceClient &client, const double &time) {
  asap::Feedback srv;
  srv.request.time = time;
  srv.request.time_min = 1 * 1e-3;
  srv.request.time_max = 9 * 1e-3;
  srv.request.size_min = 1000;
  srv.request.size_max = 8000;
  srv.request.kappa = 5;
  srv.request.gamma_min = 0.2;
  srv.request.gamma_max = 0.8;
  client.call(srv);
}

inline double saturation(const double x, const double xmax, const double xmin) {
  return (x > xmax) ? xmax : ((x < xmin) ? xmin : x);
}

#endif
