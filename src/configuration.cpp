#include "asap/asap.hpp"

bool Asap::loadConfig() {
  bool success = true;
  if(checkConfig()) {
    /*** --- DVS --- ***/
    std::string aux("");
    n_.getParam("/asap/dvs/mode", aux);
    std::unordered_map<std::string, Mode> STRING2MODE = {{"TIME", Mode::TIME}, {"SIZE", Mode::SIZE}, {"AUTO", Mode::AUTO}};
    if(STRING2MODE.count(aux)) {
      config_.dvs.mode = STRING2MODE[aux];
    } else {
      ROS_ERROR("ASAP: Error parsing configuration.");
      success = false;
    }

    switch(config_.dvs.mode) {
    case Mode::SIZE:
      n_.getParam("/asap/dvs/enabled", config_.dvs.enabled);
      n_.getParam("/asap/dvs/gamma", config_.dvs.gamma);
      n_.getParam("/asap/dvs/size", config_.dvs.size);
      config_.dvs.rate = 0;
      break;
    case Mode::TIME:
      n_.getParam("/asap/dvs/enabled", config_.dvs.enabled);
      n_.getParam("/asap/dvs/gamma", config_.dvs.gamma);
      n_.getParam("/asap/dvs/rate", config_.dvs.rate);
      config_.dvs.size = 0;
      break;
    case Mode::AUTO:
      n_.getParam("/asap/dvs/enabled", config_.dvs.enabled);
      config_.dvs.rate = 0;
      break;
    }

    /*** --- APS --- ***/
    n_.getParam("/asap/aps/enabled", config_.aps.enabled);
    n_.getParam("/asap/aps/exposure", config_.aps.exposure);
    n_.getParam("/asap/aps/rate", config_.aps.rate);

    /*** --- IMU --- ***/
    n_.getParam("/asap/imu/enabled", config_.imu.enabled);

    ROS_INFO("ASAP: New configuration loaded.");
    displayConfig();
    configDevice();
  } else {
    ROS_ERROR("ASAP: Error loading configuration.");
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
  return n_.hasParam("/asap/aps/enabled") &&
         n_.hasParam("/asap/aps/exposure") &&
         n_.hasParam("/asap/aps/rate") &&
         n_.hasParam("/asap/dvs/enabled") &&
         n_.hasParam("/asap/dvs/gamma") &&
         n_.hasParam("/asap/dvs/mode") &&
         n_.hasParam("/asap/dvs/rate") &&
         n_.hasParam("/asap/dvs/size") &&
         n_.hasParam("/asap/imu/enabled");
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
