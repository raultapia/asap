#include "asap/asap.hpp"

void Asap::initServices() {
  feedbackSrv_ = n_.advertiseService("/asap/feedback", &Asap::feedbackCallback, this);
  configurationSrv_ = n_.advertiseService("/asap/set_configuration", &Asap::setConfigurationCallback, this);
  roiSrv_ = n_.advertiseService("/asap/set_roi", &Asap::setRoiCallback, this);
  server_.setCallback(boost::bind(&Asap::dynamicReconfigureCallback, this, _1, _2));
  ROS_INFO("ASAP: Services initialized");
}

bool Asap::feedbackCallback(asap::Feedback::Request &req, asap::Feedback::Response &res) {
  if(config_.dvs.mode != Mode::AUTO) {
    res.success = false;
    return res.success;
  }

  const double tmin = req.time_min;
  const double tmax = req.time_max;
  req.time = saturation(req.time, tmax, tmin);

  /*** --- Inner loop --- ***/
  const uint32_t smin = req.size_min;
  const uint32_t smax = req.size_max;
  const double kappa = req.kappa;
  if(!(smax - smin == autoModeRecall_[0] && tmax - tmin == autoModeRecall_[1] && kappa == autoModeRecall_[2])) {
    phiFunctionScale_ = (smax - smin) / (atan(kappa * log(tmax)) - atan(kappa * log(tmin)));
    phiFunctionOffset_ = smax - phiFunctionScale_ * atan(kappa * log(tmax));
    autoModeRecall_[0] = smax - smin;
    autoModeRecall_[1] = tmax - tmin;
    autoModeRecall_[2] = kappa;
  }
  config_.dvs.size = saturation((phiFunctionScale_ * atan(kappa * log(req.time)) + phiFunctionOffset_), smax, smin);

  /*** --- Outter loop --- ***/
  gammaMax_ = req.gamma_max - ((req.time - tmin) / (tmax - tmin)) * (req.gamma_max - req.gamma_min);
  gammaMin_ = req.gamma_min;

  res.success = true;
  return res.success;
}

bool Asap::setConfigurationCallback(asap::Configuration::Request &req, asap::Configuration::Response &res) {
  if(req.dvs_mode == "TIME" || req.dvs_mode == "SIZE" || req.dvs_mode == "AUTO") {
    n_.setParam("/asap/aps/enabled", static_cast<bool>(req.aps_enabled));
    n_.setParam("/asap/aps/exposure", static_cast<int>(req.aps_exposure));
    n_.setParam("/asap/aps/rate", static_cast<int>(req.aps_rate));
    n_.setParam("/asap/dvs/enabled", static_cast<bool>(req.dvs_enabled));
    n_.setParam("/asap/dvs/gamma", static_cast<double>(req.dvs_gamma));
    n_.setParam("/asap/dvs/mode", static_cast<std::string>(req.dvs_mode));
    n_.setParam("/asap/dvs/rate", static_cast<int>(req.dvs_rate));
    n_.setParam("/asap/dvs/size", static_cast<int>(req.dvs_size));
    n_.setParam("/asap/imu/enabled", static_cast<bool>(req.imu_enabled));
    res.success = loadConfig();
  } else {
    res.success = false;
  }
  return res.success;
}

bool Asap::setRoiCallback(asap::Roi::Request &req, asap::Roi::Response &res) {
  res.success = camera_.setRoi(cv::Rect(req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height));
  return res.success;
}

void Asap::dynamicReconfigureCallback(asap::parametersConfig &config, uint32_t level) {
  if(static_cast<int>(level) < 0) {
    return;
  }
  if(config.dvs_mode == "TIME" || config.dvs_mode == "SIZE" || config.dvs_mode == "AUTO") {
    n_.setParam("/asap/aps/enabled", static_cast<bool>(config.aps_enabled));
    n_.setParam("/asap/aps/exposure", static_cast<int>(config.aps_exposure));
    n_.setParam("/asap/aps/rate", static_cast<int>(config.aps_rate));
    n_.setParam("/asap/dvs/enabled", static_cast<bool>(config.dvs_enabled));
    n_.setParam("/asap/dvs/gamma", static_cast<double>(config.dvs_gamma));
    n_.setParam("/asap/dvs/mode", static_cast<std::string>(config.dvs_mode));
    n_.setParam("/asap/dvs/rate", static_cast<int>(config.dvs_rate));
    n_.setParam("/asap/dvs/size", static_cast<int>(config.dvs_size));
    n_.setParam("/asap/imu/enabled", static_cast<bool>(config.imu_enabled));
    loadConfig();
  }
}
