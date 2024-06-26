#include "asap/asap.hpp"

Asap::Asap(ros::NodeHandle &n) : n_{n} {
  loadConfig();
  initPublishers();
  initServices();
  ROS_INFO("ASAP: Initialized.");
}

void Asap::run() {
  camera_.start();
  running_.store(true);
  ROS_INFO("ASAP: Running.");

  while(ros::ok()) {
    {
      const std::lock_guard<std::mutex> lock_dvs(dvsMutex_);
      const std::lock_guard<std::mutex> lock_aps(apsMutex_);
      const std::lock_guard<std::mutex> lock_imu(imuMutex_);

      camera_.getData(dvsBuffer_, apsBuffer_, imuBuffer_);

      if(dvsBuffer_.size() > DVS_BUFFER_WARNING_SIZE) {
        ROS_WARN("DVS buffer size exceeds recommended limit");
      }
      if(apsBuffer_.size() > APS_BUFFER_WARNING_SIZE) {
        ROS_WARN("APS buffer size exceeds recommended limit");
      }
      if(imuBuffer_.size() > IMU_BUFFER_WARNING_SIZE) {
        ROS_WARN("IMU buffer size exceeds recommended limit");
      }
    }

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  std::cout << "\nASAP: Closing threads..." << std::endl;
  running_.store(false);
  dvsThread_.join();
  apsThread_.join();
  imuThread_.join();
  std::cout << "ASAP: Closing main thread..." << std::endl;
}
