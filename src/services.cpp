#include "asap/asap.hpp"

#if ROS == 1
#define asap_Feedback_Request asap::Feedback::Request &
#define asap_Configuration_Request asap::Configuration::Request &
#define asap_Roi_Request asap::Roi::Request &
#define asap_Feedback_Response asap::Feedback::Response &
#define asap_Configuration_Response asap::Configuration::Response &
#define asap_Roi_Response asap::Roi::Response &
#define GET(x) x
#elif ROS == 2
#define asap_Feedback_Request const std::shared_ptr<asap::srv::Feedback::Request>
#define asap_Configuration_Request const std::shared_ptr<asap::srv::Configuration::Request>
#define asap_Roi_Request const std::shared_ptr<asap::srv::Roi::Request>
#define asap_Feedback_Response std::shared_ptr<asap::srv::Feedback::Response>
#define asap_Configuration_Response std::shared_ptr<asap::srv::Configuration::Response>
#define asap_Roi_Response std::shared_ptr<asap::srv::Roi::Response>
#define GET(x) (*x.get())
#endif

void Asap::initServices() {
#if ROS == 1
  feedbackSrv_ = std::make_shared<ros::ServiceServer>(n_->advertiseService("/asap/feedback", &Asap::feedbackCallback, this));
  configurationSrv_ = std::make_shared<ros::ServiceServer>(n_->advertiseService("/asap/set_configuration", &Asap::setConfigurationCallback, this));
  roiSrv_ = std::make_shared<ros::ServiceServer>(n_->advertiseService("/asap/set_roi", &Asap::setRoiCallback, this));
  server_.setCallback(boost::bind(&Asap::dynamicReconfigureCallback, this, _1, _2));
#elif ROS == 2
  feedbackSrv_ = n_->create_service<asap::srv::Feedback>("/asap/feedback", std::bind(&Asap::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2));
  configurationSrv_ = n_->create_service<asap::srv::Configuration>("/asap/set_configuration", std::bind(&Asap::setConfigurationCallback, this, std::placeholders::_1, std::placeholders::_2));
  roiSrv_ = n_->create_service<asap::srv::Roi>("/asap/set_roi", std::bind(&Asap::setRoiCallback, this, std::placeholders::_1, std::placeholders::_2));
#endif
  rosx_log_info("ASAP: Services initialized");
}

bool Asap::feedbackCallback(asap_Feedback_Request req, asap_Feedback_Response res) {
  if(config_.dvs.mode != Mode::AUTO) {
    GET(res).success = false;
    return GET(res).success;
  }

  const double tmin = GET(req).time_min;
  const double tmax = GET(req).time_max;
  const double t = saturation(GET(req).time, tmax, tmin);

  /*** --- Inner loop --- ***/
  const uint32_t smin = GET(req).size_min;
  const uint32_t smax = GET(req).size_max;
  const double kappa = GET(req).kappa;
  if(!(smax - smin == autoModeRecall_[0] && tmax - tmin == autoModeRecall_[1] && kappa == autoModeRecall_[2])) {
    phiFunctionScale_ = (smax - smin) / (atan(kappa * log(tmax)) - atan(kappa * log(tmin)));
    phiFunctionOffset_ = smax - phiFunctionScale_ * atan(kappa * log(tmax));
    autoModeRecall_[0] = smax - smin;
    autoModeRecall_[1] = tmax - tmin;
    autoModeRecall_[2] = kappa;
  }
  config_.dvs.size = saturation((phiFunctionScale_ * atan(kappa * log(t)) + phiFunctionOffset_), smax, smin);

  /*** --- Outter loop --- ***/
  gammaMax_ = GET(req).gamma_max - ((t - tmin) / (tmax - tmin)) * (GET(req).gamma_max - GET(req).gamma_min);
  gammaMin_ = GET(req).gamma_min;

  GET(res).success = true;
  return GET(res).success;
}

bool Asap::setConfigurationCallback(asap_Configuration_Request req, asap_Configuration_Response res) {
  if(GET(req).dvs_mode == "TIME" || GET(req).dvs_mode == "SIZE" || GET(req).dvs_mode == "AUTO") {
#if ROS == 1
    n_->setParam("aps/enabled", static_cast<bool>(GET(req).aps_enabled));
    n_->setParam("aps/exposure", static_cast<int>(GET(req).aps_exposure));
    n_->setParam("aps/rate", static_cast<int>(GET(req).aps_rate));
    n_->setParam("dvs/enabled", static_cast<bool>(GET(req).dvs_enabled));
    n_->setParam("dvs/gamma", static_cast<double>(GET(req).dvs_gamma));
    n_->setParam("dvs/mode", static_cast<std::string>(GET(req).dvs_mode));
    n_->setParam("dvs/rate", static_cast<int>(GET(req).dvs_rate));
    n_->setParam("dvs/size", static_cast<int>(GET(req).dvs_size));
    n_->setParam("imu/enabled", static_cast<bool>(GET(req).imu_enabled));
#elif ROS == 2
    n_->set_parameter(rclcpp::Parameter("aps/enabled", static_cast<bool>(GET(req).aps_enabled)));
    n_->set_parameter(rclcpp::Parameter("aps/exposure", static_cast<int>(GET(req).aps_exposure)));
    n_->set_parameter(rclcpp::Parameter("aps/rate", static_cast<int>(GET(req).aps_rate)));
    n_->set_parameter(rclcpp::Parameter("dvs/enabled", static_cast<bool>(GET(req).dvs_enabled)));
    n_->set_parameter(rclcpp::Parameter("dvs/gamma", static_cast<double>(GET(req).dvs_gamma)));
    n_->set_parameter(rclcpp::Parameter("dvs/mode", static_cast<std::string>(GET(req).dvs_mode)));
    n_->set_parameter(rclcpp::Parameter("dvs/rate", static_cast<int>(GET(req).dvs_rate)));
    n_->set_parameter(rclcpp::Parameter("dvs/size", static_cast<int>(GET(req).dvs_size)));
    n_->set_parameter(rclcpp::Parameter("imu/enabled", static_cast<bool>(GET(req).imu_enabled)));
#endif
    GET(res).success = loadConfig();
  } else {
    GET(res).success = false;
  }
  return GET(res).success;
}

bool Asap::setRoiCallback(asap_Roi_Request req, asap_Roi_Response res) {
  GET(res).success = camera_.setRoi(cv::Rect(GET(req).roi.x_offset, GET(req).roi.y_offset, GET(req).roi.width, GET(req).roi.height));
  return GET(res).success;
}

#if ROS == 1
void Asap::dynamicReconfigureCallback(asap::parametersConfig &config, uint32_t level) {
  if(static_cast<int>(level) < 0) {
    return;
  }
  if(config.dvs_mode == "TIME" || config.dvs_mode == "SIZE" || config.dvs_mode == "AUTO") {
#if ROS == 1
    n_->setParam("aps/enabled", static_cast<bool>(config.aps_enabled));
    n_->setParam("aps/exposure", static_cast<int>(config.aps_exposure));
    n_->setParam("aps/rate", static_cast<int>(config.aps_rate));
    n_->setParam("dvs/enabled", static_cast<bool>(config.dvs_enabled));
    n_->setParam("dvs/gamma", static_cast<double>(config.dvs_gamma));
    n_->setParam("dvs/mode", static_cast<std::string>(config.dvs_mode));
    n_->setParam("dvs/rate", static_cast<int>(config.dvs_rate));
    n_->setParam("dvs/size", static_cast<int>(config.dvs_size));
    n_->setParam("imu/enabled", static_cast<bool>(config.imu_enabled));
#elif ROS == 2
    n_->set_parameter(rclcpp::Parameter("aps/enabled", static_cast<bool>(config.aps_enabled)));
    n_->set_parameter(rclcpp::Parameter("aps/exposure", static_cast<int>(config.aps_exposure)));
    n_->set_parameter(rclcpp::Parameter("aps/rate", static_cast<int>(config.aps_rate)));
    n_->set_parameter(rclcpp::Parameter("dvs/enabled", static_cast<bool>(config.dvs_enabled)));
    n_->set_parameter(rclcpp::Parameter("dvs/gamma", static_cast<double>(config.dvs_gamma)));
    n_->set_parameter(rclcpp::Parameter("dvs/mode", static_cast<std::string>(config.dvs_mode)));
    n_->set_parameter(rclcpp::Parameter("dvs/rate", static_cast<int>(config.dvs_rate)));
    n_->set_parameter(rclcpp::Parameter("dvs/size", static_cast<int>(config.dvs_size)));
    n_->set_parameter(rclcpp::Parameter("imu/enabled", static_cast<bool>(config.imu_enabled)));
#endif
    loadConfig();
  }
}
#endif
