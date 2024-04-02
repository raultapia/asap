#include "asap/asap.hpp"

void Asap::initPublishers() {
  if(config_.dvs.enabled) {
    dvsPub_ = n_.advertise<dvs_msgs::EventArray>("/asap/events", 100);
    dvsThread_ = std::thread(&Asap::dvsPublishFunction, this);
  }

  if(config_.aps.enabled) {
    apsPub_ = n_.advertise<sensor_msgs::Image>("/asap/image_raw", 10);
    apsThread_ = std::thread(&Asap::apsPublishFunction, this);
  }

  if(config_.imu.enabled) {
    imuPub_ = n_.advertise<sensor_msgs::Imu>("/asap/imu", 100);
    imuThread_ = std::thread(&Asap::imuPublishFunction, this);
  }

  if(config_.dvs.enabled || config_.aps.enabled) {
    cameraInfoPub_ = n_.advertise<sensor_msgs::CameraInfo>("/asap/camera_info", 1);
    cameraInfoThread_ = std::thread(&Asap::cameraInfoPublishFunction, this);
  }

  ROS_INFO("ASAP: Publishers initialized");
}

void Asap::dvsPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::Event e;
  dvs_msgs::Event emsg;
  dvs_msgs::EventArray amsg;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  while(running_.load()) {
    if(!config_.dvs.enabled || dvsBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
    if(dvsPub_.getNumSubscribers() < 1){
      dvsMutex_.lock();
      while(!dvsBuffer_.empty()){
        dvsBuffer_.pop();
      }
      dvsMutex_.unlock();
      continue;
    }

    switch(config_.dvs.mode) {
    case Mode::SIZE:
    case Mode::AUTO:
      dvsMutex_.lock();
      while(!dvsBuffer_.empty() && amsg.events.size() < static_cast<std::size_t>(config_.dvs.size)) {
        if(config_.dvs.gamma == 1 || distribution(generator) < config_.dvs.gamma) {
          e = dvsBuffer_.front();
          emsg.x = e.x;
          emsg.y = e.y;
          emsg.ts.fromSec(e.t);
          emsg.polarity = e.p;
          amsg.events.push_back(emsg);
        }
        dvsBuffer_.pop();
      }
      dvsMutex_.unlock();
      break;
    case Mode::TIME:
      dvsMutex_.lock();
      while(!dvsBuffer_.empty() && timediff(amsg.events) < hz2sec(config_.dvs.rate)) {
        if(config_.dvs.gamma == 1 || distribution(generator) < config_.dvs.gamma) {
          e = dvsBuffer_.front();
          emsg.x = e.x;
          emsg.y = e.y;
          emsg.ts.fromSec(e.t);
          emsg.polarity = e.p;
          amsg.events.push_back(emsg);
        }
        dvsBuffer_.pop();
      }
      dvsMutex_.unlock();
      break;
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

    amsg.header.stamp = ros::Time::now();
    amsg.height = camera_.getSensorSize().height;
    amsg.width = camera_.getSensorSize().width;
    dvsPub_.publish(amsg);
    amsg.events.clear();
  }
}

void Asap::apsPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::StampedMatVector aps;
  sensor_msgs::Image msg;
  std_msgs::Header header;
  header.frame_id = "/cam";

  while(running_.load()) {
    if(!config_.aps.enabled || apsBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
    if(apsPub_.getNumSubscribers() < 1){
      apsMutex_.lock();
      while(!apsBuffer_.empty()){
        apsBuffer_.pop();
      }
      apsMutex_.unlock();
      continue;
    }

    apsMutex_.lock();
    while(!apsBuffer_.empty()) {
      aps.emplace_back(apsBuffer_.front());
      apsBuffer_.pop();
    }
    apsMutex_.unlock();
    for(const ev::StampedMat &a : aps) {
      header.stamp.fromSec(a.t);
      cv_bridge::CvImage br = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, a);
      br.toImageMsg(msg);
      apsPub_.publish(msg);
    }
    aps.clear();
  }
}

void Asap::imuPublishFunction() {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  ev::ImuVector imu;
  sensor_msgs::Imu msg;
  msg.header.frame_id = "/cam";
  msg.orientation_covariance[0] = -1;

  while(running_.load()) {
    if(!config_.imu.enabled || imuBuffer_.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
    if(imuPub_.getNumSubscribers() < 1){
      imuMutex_.lock();
      while(!imuBuffer_.empty()){
        imuBuffer_.pop();
      }
      imuMutex_.unlock();
      continue;
    }

    imuMutex_.lock();
    while(!imuBuffer_.empty()) {
      imu.emplace_back(imuBuffer_.front());
      imuBuffer_.pop();
    }
    imuMutex_.unlock();

    for(const ev::Imu &i : imu) {
      msg.header.stamp.fromSec(i.t);
      msg.linear_acceleration.x = i.linear_acceleration.x;
      msg.linear_acceleration.y = i.linear_acceleration.y;
      msg.linear_acceleration.z = i.linear_acceleration.z;
      msg.angular_velocity.x = i.angular_velocity.x;
      msg.angular_velocity.y = i.angular_velocity.y;
      msg.angular_velocity.z = i.angular_velocity.z;
      imuPub_.publish(msg);
    }
    imu.clear();
  }
}

void Asap::cameraInfoPublishFunction() const {
  while(!running_.load()) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
  }

  sensor_msgs::CameraInfo msg;
  ros::Rate rate(2);

  while(running_.load()) {
    if(!config_.dvs.enabled && !config_.aps.enabled) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(THREAD_SLEEP_TIME_NSEC));
      continue;
    }
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/cam";
    msg.width = camera_.getSensorSize().width;
    msg.height = camera_.getSensorSize().height;
    cameraInfoPub_.publish(msg);
    rate.sleep();
  }
}
