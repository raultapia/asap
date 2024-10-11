#include "asap/asap.hpp"

bool Asap::loadConfig() {
  bool success = true;
  if(checkConfig()) {
    /*** --- DVS --- ***/
    std::string aux("");
#if ROS == 1
    n_->getParam("dvs/mode", aux);
#elif ROS == 2
    n_->get_parameter("dvs/mode", aux);
#endif
    std::unordered_map<std::string, Mode> STRING2MODE = {{"TIME", Mode::TIME}, {"SIZE", Mode::SIZE}, {"AUTO", Mode::AUTO}};
    if(STRING2MODE.count(aux)) {
      config_.dvs.mode = STRING2MODE[aux];
    } else {
      rosx_log_error("ASAP: Error parsing configuration.");
      success = false;
    }

    switch(config_.dvs.mode) {
    case Mode::SIZE:
#if ROS == 1
      n_->getParam("dvs/enabled", config_.dvs.enabled);
      n_->getParam("dvs/gamma", config_.dvs.gamma);
      n_->getParam("dvs/size", config_.dvs.size);
#elif ROS == 2
      n_->get_parameter("dvs/enabled", config_.dvs.enabled);
      n_->get_parameter("dvs/gamma", config_.dvs.gamma);
      n_->get_parameter("dvs/size", config_.dvs.size);
#endif
      config_.dvs.rate = 0;
      break;
    case Mode::TIME:
#if ROS == 1
      n_->getParam("dvs/enabled", config_.dvs.enabled);
      n_->getParam("dvs/gamma", config_.dvs.gamma);
      n_->getParam("dvs/rate", config_.dvs.rate);
#elif ROS == 2
      n_->get_parameter("dvs/enabled", config_.dvs.enabled);
      n_->get_parameter("dvs/gamma", config_.dvs.gamma);
      n_->get_parameter("dvs/rate", config_.dvs.rate);
#endif
      config_.dvs.size = 0;
      break;
    case Mode::AUTO:
#if ROS == 1
      n_->getParam("dvs/enabled", config_.dvs.enabled);
#elif ROS == 2
      n_->get_parameter("dvs/enabled", config_.dvs.enabled);
#endif
      config_.dvs.rate = 0;
      break;
    }

#if ROS == 1
    /*** --- APS --- ***/
    n_->getParam("aps/enabled", config_.aps.enabled);
    n_->getParam("aps/exposure", config_.aps.exposure);
    n_->getParam("aps/rate", config_.aps.rate);
#elif ROS == 2
    /*** --- APS --- ***/
    n_->get_parameter("aps/enabled", config_.aps.enabled);
    n_->get_parameter("aps/exposure", config_.aps.exposure);
    n_->get_parameter("aps/rate", config_.aps.rate);
#endif

#if ROS == 1
    /*** --- IMU --- ***/
    n_->getParam("imu/enabled", config_.imu.enabled);
#elif ROS == 2
    /*** --- IMU --- ***/
    n_->get_parameter("imu/enabled", config_.imu.enabled);
#endif

    rosx_log_info("ASAP: New configuration loaded.");
    displayConfig();
    configDevice();
  } else {
    rosx_log_error("ASAP: Error loading configuration.");
    success = false;
  }
  return success;
}

void Asap::configDevice() {
  // /*** --- Select data to sent --- ***/
  camera_.enableDvs(config_.dvs.enabled);
  camera_.enableAps(config_.aps.enabled);
  camera_.enableImu(config_.imu.enabled);

  // /*** --- DVS configuration --- ***/
  switch(config_.dvs.mode) {
  case Mode::SIZE:
    camera_.setDvsTimeInterval(0);
    camera_.setDvsEventsPerPacket(config_.dvs.size);
    break;
  case Mode::TIME:
    camera_.setDvsTimeInterval(hz2usec(config_.dvs.rate));
    camera_.setDvsEventsPerPacket(0);
    break;
  case Mode::AUTO:
    camera_.setDvsTimeInterval(hz2usec(DVS_DEFAULT_RATE));
    camera_.setDvsEventsPerPacket(DVS_DEFAULT_SIZE);
    break;
  }

  // /*** --- APS configuration --- ***/
  camera_.setApsTimeInterval(hz2usec(config_.aps.rate));
  camera_.setExposure(config_.aps.exposure);
}

bool Asap::checkConfig() const {
#if ROS == 1
  return n_->hasParam("aps/enabled") &&
         n_->hasParam("aps/exposure") &&
         n_->hasParam("aps/rate") &&
         n_->hasParam("dvs/enabled") &&
         n_->hasParam("dvs/gamma") &&
         n_->hasParam("dvs/mode") &&
         n_->hasParam("dvs/rate") &&
         n_->hasParam("dvs/size") &&
         n_->hasParam("imu/enabled");
#elif ROS == 2
  return n_->has_parameter("aps/enabled") &&
         n_->has_parameter("aps/exposure") &&
         n_->has_parameter("aps/rate") &&
         n_->has_parameter("dvs/enabled") &&
         n_->has_parameter("dvs/gamma") &&
         n_->has_parameter("dvs/mode") &&
         n_->has_parameter("dvs/rate") &&
         n_->has_parameter("dvs/size") &&
         n_->has_parameter("imu/enabled");
#endif
}

void Asap::displayConfig() const {
  const std::string dvs_check = (config_.dvs.enabled ? "\u001b[32m [✓]\u001b[0m " : "\u001b[31m [✗]\u001b[0m ");
  const std::string aps_check = (config_.aps.enabled ? "\u001b[32m [✓]\u001b[0m " : "\u001b[31m [✗]\u001b[0m ");
  const std::string imu_check = (config_.imu.enabled ? "\u001b[32m [✓]\u001b[0m " : "\u001b[31m [✗]\u001b[0m ");
  const std::string camera_info_check = (config_.dvs.enabled || config_.aps.enabled ? "\u001b[32m [✓]\u001b[0m " : "\u001b[31m [✗]\u001b[0m ");

  std::cout << "-----" << std::endl;

  std::cout << dvs_check << "DVS publisher";
  if(config_.dvs.enabled) {
    switch(config_.dvs.mode) {
    case Mode::SIZE:
      std::cout << " @ " << config_.dvs.size << " events/packet" << std::endl;
      break;
    case Mode::TIME:
      std::cout << " @ " << config_.dvs.rate << " Hz" << std::endl;
      break;
    case Mode::AUTO:
      std::cout << " @ auto" << std::endl;
      break;
    }
  } else {
    std::cout << std::endl;
  }

  std::cout << aps_check << "APS publisher";
  if(config_.aps.enabled) {
    std::cout << " @ " << config_.aps.rate << " Hz" << std::endl;
  } else {
    std::cout << std::endl;
  }

  std::cout << imu_check << "IMU publisher" << std::endl;

  std::cout << camera_info_check << "Camera info publisher @ 1 Hz" << std::endl;

  std::cout << " - Camera exposure: " << config_.aps.exposure << std::endl;

  if(config_.dvs.mode == Mode::AUTO)
    std::cout << " - Gamma filter enabled" << std::endl;
  else if(config_.dvs.gamma != 1)
    std::cout << " - Gamma filter: " << config_.dvs.gamma << std::endl;
  else
    std::cout << " - Gamma filter disabled" << std::endl;

  std::cout << "-----" << std::endl;
}
