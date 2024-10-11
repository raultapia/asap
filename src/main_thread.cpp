#include "asap/asap.hpp"

#if ROS == 1
Asap::Asap(std::shared_ptr<ros::NodeHandle> n) : n_{n} {
#elif ROS == 2
Asap::Asap(rclcpp::Node::SharedPtr n) : n_{n} {
  n_->declare_parameter("aps/enabled", false);
  n_->declare_parameter("aps/exposure", 0);
  n_->declare_parameter("aps/rate", 0.0);
  n_->declare_parameter("dvs/enabled", true);
  n_->declare_parameter("dvs/gamma", 0.0);
  n_->declare_parameter("dvs/mode", "SIZE");
  n_->declare_parameter("dvs/rate", 0.0);
  n_->declare_parameter("dvs/size", 1000);
  n_->declare_parameter("imu/enabled", false);
#endif
  std::this_thread::sleep_for(std::chrono::seconds(1));
  loadConfig();
  initPublishers();
  initServices();
  rosx_log_info("ASAP: Initialized.");
}

void Asap::run() {
  camera_.start();
  running_.store(true);
  rosx_log_info("ASAP: Running.");

  while(
#if ROS == 1
      ros::ok()
#elif ROS == 2
      rclcpp::ok()
#endif
  ) {
    {
      const std::lock_guard<std::mutex> lock_dvs(dvsMutex_);
      const std::lock_guard<std::mutex> lock_aps(apsMutex_);
      const std::lock_guard<std::mutex> lock_imu(imuMutex_);

      camera_.getData(dvsBuffer_, apsBuffer_, imuBuffer_);

      if(dvsBuffer_.size() > DVS_BUFFER_WARNING_SIZE) {
        rosx_log_warn("DVS buffer size exceeds recommended limit");
        std::cout << dvsBuffer_.size() << std::endl;
      }
      if(apsBuffer_.size() > APS_BUFFER_WARNING_SIZE) {
        rosx_log_warn("APS buffer size exceeds recommended limit");
        std::cout << apsBuffer_.size() << std::endl;
      }
      if(imuBuffer_.size() > IMU_BUFFER_WARNING_SIZE) {
        rosx_log_warn("IMU buffer size exceeds recommended limit");
        std::cout << imuBuffer_.size() << std::endl;
      }
    }

#if ROS == 1
    ros::spinOnce();
#elif ROS == 2
    rclcpp::spin_some(n_);
#endif
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  std::cout << "\nASAP: Closing threads..." << std::endl;
  running_.store(false);
  dvsThread_.join();
  apsThread_.join();
  imuThread_.join();
  std::cout << "ASAP: Closing main thread..." << std::endl;
}
