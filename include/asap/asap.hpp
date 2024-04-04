#ifndef ASAP_ASAP_HPP
#define ASAP_ASAP_HPP

#include <asap/Configuration.h>
#include <asap/Feedback.h>
#include <asap/Roi.h>
#include <asap/parametersConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <dynamic_reconfigure/server.h>
#include <openev/devices.hpp>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <unordered_map>

enum class Mode : uint8_t { TIME,
                            SIZE,
                            AUTO };

constexpr double GAMMA_FILTER_ALPHA = 0.99;
constexpr double GAMMA_FILTER_1_DIV_ALPHA = 1 / GAMMA_FILTER_ALPHA;
constexpr double INIT_RATE = 0.5 * 1e6;
constexpr size_t EVENT_RATE_QUEUE_MAX_SIZE = 500;
constexpr size_t DVS_BUFFER_WARNING_SIZE = 1e8;
constexpr size_t APS_BUFFER_WARNING_SIZE = 50;
constexpr size_t IMU_BUFFER_WARNING_SIZE = 300;
constexpr int THREAD_SLEEP_TIME_NSEC = 10;

constexpr int APS_DEFAULT_EXPOSURE = 6500;
constexpr double APS_DEFAULT_RATE = 50;
constexpr double DVS_DEFAULT_GAMMA = 1;
constexpr double DVS_DEFAULT_RATE = 50;
constexpr int DVS_DEFAULT_SIZE = 1000;
constexpr Mode DVS_DEFAULT_MODE = Mode::SIZE;

struct DvsConfiguration {
  bool enabled{true};
  Mode mode{DVS_DEFAULT_MODE};
  int size{DVS_DEFAULT_SIZE};
  double rate{DVS_DEFAULT_RATE};
  double gamma{DVS_DEFAULT_GAMMA};
} __attribute__((aligned(32)));

struct ApsConfiguration {
  bool enabled{true};
  double rate{APS_DEFAULT_RATE};
  int exposure{APS_DEFAULT_EXPOSURE};
} __attribute__((aligned(16)));

struct ImuConfiguration {
  bool enabled{true};
};

struct Configuration {
  DvsConfiguration dvs;
  ApsConfiguration aps;
  ImuConfiguration imu;
} __attribute__((aligned(128)));

class Asap {
public:
  Asap() = delete;
  explicit Asap(ros::NodeHandle &n);
  ~Asap() = default;
  Asap(const Asap &) = delete;
  Asap(Asap &&o) = delete;
  Asap &operator=(const Asap &) = delete;
  Asap &operator=(Asap &&) = delete;
  void run();

private:
  ros::NodeHandle &n_;
  ev::Davis346 camera_;
  ev::Queue dvsBuffer_{};
  ev::StampedMatQueue apsBuffer_{};
  ev::ImuQueue imuBuffer_{};
  std::mutex dvsMutex_{}, apsMutex_{}, imuMutex_{};
  std::atomic<bool> running_;
  std::thread dvsThread_, apsThread_, imuThread_, cameraInfoThread_;
  ros::Publisher dvsPub_{}, apsPub_{}, imuPub_{}, cameraInfoPub_{};
  ros::ServiceServer feedbackSrv_{}, configurationSrv_{}, roiSrv_{};
  dynamic_reconfigure::Server<asap::parametersConfig> server_{};
  std::array<double, 3> autoModeRecall_{0, 0, 0};
  std::queue<double> timeQueue_{};
  double gammaMax_{};
  double gammaMin_{};
  double rate_{};
  double rateMax_{INIT_RATE};
  double rateMin_{INIT_RATE};
  double phiFunctionScale_{1};
  double phiFunctionOffset_{0};
  Configuration config_;
  bool loadConfig();
  [[nodiscard]] bool checkConfig() const;
  void displayConfig() const;
  void configDevice();
  void initPublishers();
  void dvsPublishFunction();
  void apsPublishFunction();
  void imuPublishFunction();
  void cameraInfoPublishFunction() const;
  void initServices();
  bool feedbackCallback(asap::Feedback::Request &, asap::Feedback::Response &);
  bool setConfigurationCallback(asap::Configuration::Request &, asap::Configuration::Response &);
  bool setRoiCallback(asap::Roi::Request &, asap::Roi::Response &);
  void dynamicReconfigureCallback(asap::parametersConfig &config, uint32_t level);
};

inline double saturation(const double x, const double xmax, const double xmin) {
  return (x > xmax) ? xmax : ((x < xmin) ? xmin : x);
}

inline double timediff(const std::vector<dvs_msgs::Event> &x) {
  return (x.size() < 2) ? 0 : x.back().ts.toSec() - x.front().ts.toSec();
}

inline double timediff(const std::queue<double> &x) {
  return (x.size() < 2) ? 0 : x.back() - x.front();
}

inline double hz2sec(const double &x) {
  return (1.0 / x);
}

inline uint32_t hz2usec(const double &x) {
  return static_cast<uint32_t>(1e6 * (1.0 / x));
}

#endif // ASAP_ASAP_HPP
