#include "asap/asap.hpp"

template <typename T>
inline void clear_buffer(std::queue<T> &q, std::mutex &m) {
  const std::lock_guard<std::mutex> lock(m);
  while(!q.empty()) {
    q.pop();
  }
}

template <typename T>
inline void get_data_from_buffer(std::queue<T> &q, std::vector<T> &v, std::mutex &m) {
  const std::lock_guard<std::mutex> lock(m);
  while(!q.empty()) {
    v.emplace_back(q.front());
    q.pop();
  }
}

void Asap::initPublishers() {
  if(config_.dvs.enabled) {
#if ROS == 1
    dvsPub_ = std::make_shared<ros::Publisher>(n_->advertise<dvs_msgs::EventArray>("/asap/events", 100));
#elif ROS == 2
    dvsPub_ = n_->create_publisher<dvs_msgs::msg::EventArray>("/asap/events", 100);
#endif
    dvsThread_ = std::thread(&Asap::dvsPublishFunction, this);
  }

  if(config_.aps.enabled) {
#if ROS == 1
    apsPub_ = std::make_shared<ros::Publisher>(n_->advertise<sensor_msgs::Image>("/asap/image_raw", 10));
#elif ROS == 2
    apsPub_ = n_->create_publisher<sensor_msgs::msg::Image>("/asap/image_raw", 10);
#endif
    apsThread_ = std::thread(&Asap::apsPublishFunction, this);
  }

  if(config_.imu.enabled) {
#if ROS == 1
    imuPub_ = std::make_shared<ros::Publisher>(n_->advertise<sensor_msgs::Imu>("/asap/imu", 100));
#elif ROS == 2
    imuPub_ = n_->create_publisher<sensor_msgs::msg::Imu>("/asap/imu", 100);
#endif
    imuThread_ = std::thread(&Asap::imuPublishFunction, this);
  }

  if(config_.dvs.enabled || config_.aps.enabled) {
#if ROS == 1
    cameraInfoPub_ = std::make_shared<ros::Publisher>(n_->advertise<sensor_msgs::CameraInfo>("/asap/camera_info", 1));
#elif ROS == 2
    cameraInfoPub_ = n_->create_publisher<sensor_msgs::msg::CameraInfo>("/asap/camera_info", 1);
#endif
    cameraInfoThread_ = std::thread(&Asap::cameraInfoPublishFunction, this);
  }
  rosx_log_info("ASAP: Publishers initialized");
}

void Asap::dvsPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::Event e;
#if ROS == 1
  dvs_msgs::Event emsg;
  dvs_msgs::EventArray amsg;
#elif ROS == 2
  dvs_msgs::msg::Event emsg;
  dvs_msgs::msg::EventArray amsg;
#endif
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  while(running_.load()) {
    if(!config_.dvs.enabled || dvsBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
#if ROS == 1
    if(dvsPub_->getNumSubscribers() < 1) {
#elif ROS == 2
    if(dvsPub_->get_subscription_count() < 1) {
#endif
      clear_buffer(dvsBuffer_, dvsMutex_);
      continue;
    }

    switch(config_.dvs.mode) {
    case Mode::SIZE:
    case Mode::AUTO: {
      const std::lock_guard<std::mutex> lock(dvsMutex_);
      while(!dvsBuffer_.empty() && amsg.events.size() < static_cast<std::size_t>(config_.dvs.size)) {
        if(config_.dvs.gamma == 1 || distribution(generator) < config_.dvs.gamma) {
          e = dvsBuffer_.front();
          emsg.x = e.x;
          emsg.y = e.y;
#if ROS == 1
          emsg.ts.fromSec(e.t);
#elif ROS == 2
          emsg.ts = rclcpp::Time(e.t);
#endif
          emsg.polarity = e.p;
          amsg.events.push_back(emsg);
        }
        dvsBuffer_.pop();
      }
    } break;
    case Mode::TIME: {
      const std::lock_guard<std::mutex> lock(dvsMutex_);
      while(!dvsBuffer_.empty() && timediff(amsg.events) < hz2sec(config_.dvs.rate)) {
        if(config_.dvs.gamma == 1 || distribution(generator) < config_.dvs.gamma) {
          e = dvsBuffer_.front();
          emsg.x = e.x;
          emsg.y = e.y;
#if ROS == 1
          emsg.ts.fromSec(e.t);
#elif ROS == 2
          emsg.ts = rclcpp::Time(e.t);
#endif
          emsg.polarity = e.p;
          amsg.events.push_back(emsg);
        }
        dvsBuffer_.pop();
      }
    } break;
    }

    if(config_.dvs.mode == Mode::AUTO) {
      timeQueue_.push(e.t);
      if(timeQueue_.size() > EVENT_RATE_QUEUE_MAX_SIZE) {
        timeQueue_.pop();
        rate_ = timeQueue_.size() / timediff(timeQueue_);
        rateMax_ = (rate_ > rateMax_) ? rate_ : GAMMA_FILTER_ALPHA * rateMax_;
        rateMin_ = (rate_ < rateMin_) ? rate_ : GAMMA_FILTER_1_DIV_ALPHA * rateMin_;
        if(rateMax_ == rateMin_) {
          config_.dvs.gamma = gammaMax_;
        } else {
          config_.dvs.gamma = gammaMax_ - ((rate_ - rateMin_) / (rateMax_ - rateMin_)) * (gammaMax_ - gammaMin_);
        }
      }
    }
    if(config_.dvs.mode == Mode::TIME && timediff(amsg.events) < hz2sec(config_.dvs.rate)) {
      continue;
    }
    if((config_.dvs.mode == Mode::SIZE || config_.dvs.mode == Mode::AUTO) && amsg.events.size() < static_cast<std::size_t>(config_.dvs.size)) {
      continue;
    }

#if ROS == 1
    amsg.header.stamp = ros::Time::now();
#elif ROS == 2
    amsg.header.stamp = n_->now();
#endif
    amsg.height = camera_.getSensorSize().height;
    amsg.width = camera_.getSensorSize().width;
    dvsPub_->publish(amsg);
    amsg.events.clear();
  }
}

void Asap::apsPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::StampedMatVector aps;
#if ROS == 1
  sensor_msgs::Image msg;
  std_msgs::Header header;
#elif ROS == 2
  sensor_msgs::msg::Image msg;
  std_msgs::msg::Header header;
#endif
  header.frame_id = "cam";

  while(running_.load()) {
    if(!config_.aps.enabled || apsBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
#if ROS == 1
    if(apsPub_->getNumSubscribers() < 1) {
#elif ROS == 2
    if(apsPub_->get_subscription_count() < 1) {
#endif
      clear_buffer(apsBuffer_, apsMutex_);
      continue;
    }

    get_data_from_buffer(apsBuffer_, aps, apsMutex_);
    for(const ev::StampedMat &a : aps) {
#if ROS == 1
      header.stamp.fromSec(a.t);
#elif ROS == 2
      header.stamp = rclcpp::Time(a.t);
#endif
      cv_bridge::CvImage br = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, a);
      br.toImageMsg(msg);
      apsPub_->publish(msg);
    }
    aps.clear();
  }
}

void Asap::imuPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::ImuVector imu;
#if ROS == 1
  sensor_msgs::Imu msg;
#elif ROS == 2
  sensor_msgs::msg::Imu msg;
#endif
  msg.header.frame_id = "/cam";
  msg.orientation_covariance[0] = -1;

  while(running_.load()) {
    if(!config_.imu.enabled || imuBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
#if ROS == 1
    if(imuPub_->getNumSubscribers() < 1) {
#elif ROS == 2
    if(imuPub_->get_subscription_count() < 1) {
#endif
      clear_buffer(imuBuffer_, imuMutex_);
      continue;
    }

    get_data_from_buffer(imuBuffer_, imu, imuMutex_);
    for(const ev::Imu &i : imu) {
#if ROS == 1
      msg.header.stamp.fromSec(i.t);
#elif ROS == 2
      msg.header.stamp = rclcpp::Time(i.t);
#endif
      msg.linear_acceleration.x = i.linear_acceleration.x;
      msg.linear_acceleration.y = i.linear_acceleration.y;
      msg.linear_acceleration.z = i.linear_acceleration.z;
      msg.angular_velocity.x = i.angular_velocity.x;
      msg.angular_velocity.y = i.angular_velocity.y;
      msg.angular_velocity.z = i.angular_velocity.z;
      imuPub_->publish(msg);
    }
    imu.clear();
  }
}

void Asap::cameraInfoPublishFunction() const {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

#if ROS == 1
  sensor_msgs::CameraInfo msg;
  ros::Rate rate(2);
#elif ROS == 2
  sensor_msgs::msg::CameraInfo msg;
  rclcpp::Rate rate(2);
#endif

  while(running_.load()) {
    if(!config_.dvs.enabled && !config_.aps.enabled) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
#if ROS == 1
    msg.header.stamp = ros::Time::now();
#elif ROS == 2
    msg.header.stamp = n_->now();
#endif
    msg.header.frame_id = "/cam";
    msg.width = camera_.getSensorSize().width;
    msg.height = camera_.getSensorSize().height;
    cameraInfoPub_->publish(msg);
    rate.sleep();
  }
}
