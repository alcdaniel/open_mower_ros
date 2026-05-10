#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/system/error_code.hpp>
#include <nlohmann/json.hpp>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <mower_map/MapArea.h>
#include <mower_map/AddMowingAreaSrv.h>
#include <mower_map/ClearMapSrv.h>
#include <mower_map/SetDockingPointSrv.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/Power.h>
#include <mower_msgs/Status.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <nmea_msgs/Sentence.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <xbot_msgs/AbsolutePose.h>
#include <xbot_rpc/RpcError.h>
#include <xbot_rpc/RpcRequest.h>
#include <xbot_rpc/RpcResponse.h>

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;
using json = nlohmann::json;

namespace {

double wallNowSec() {
  return ros::WallTime::now().toSec();
}

std::int64_t nowMs() {
  return static_cast<std::int64_t>(wallNowSec() * 1000.0);
}

double clampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

bool rosBoolParam(const ros::NodeHandle& nh, const std::string& name, bool default_value) {
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(name, value)) return default_value;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) return static_cast<bool>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value) != 0;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
    std::string s = static_cast<std::string>(value);
    for (char& c : s) c = static_cast<char>(std::tolower(c));
    return s == "1" || s == "true" || s == "yes" || s == "on";
  }
  return default_value;
}

std::string jsonString(const json& value) {
  return value.dump(-1, ' ', false, json::error_handler_t::replace);
}

std::vector<std::string> splitCsv(const std::string& input) {
  std::vector<std::string> out;
  std::string item;
  std::stringstream ss(input);
  while (std::getline(ss, item, ',')) {
    std::string trimmed;
    for (char c : item) {
      if (!std::isspace(static_cast<unsigned char>(c))) trimmed.push_back(c);
    }
    if (!trimmed.empty()) out.push_back(trimmed);
  }
  return out;
}

struct PointXY {
  double x = 0.0;
  double y = 0.0;
};

struct RecordingSession {
  std::string mode;
  std::vector<PointXY> points;
  double started_at = 0.0;
  double sampled_at = 0.0;
  int rtk_lost_count = 0;
  std::string last_fix = "none";
  bool paused = false;
  std::optional<std::string> pause_reason;
  double paused_at = 0.0;
  bool has_paused_at = false;
  double bad_fix_since = 0.0;
  bool has_bad_fix_since = false;
};

struct BufferedRecording {
  std::string mode;
  std::vector<PointXY> points;
};

struct RpcPending {
  std::mutex mutex;
  std::condition_variable cv;
  bool done = false;
  bool error = false;
  std::string result;
  int code = 0;
  std::string message;
};

struct StateSnapshot {
  double started_at = 0.0;
  bool manual_active = false;
  double manual_until = 0.0;

  bool has_high_level = false;
  bool has_low_level = false;
  bool has_power = false;
  bool has_emergency = false;
  bool has_left_esc = false;
  bool has_right_esc = false;
  bool has_pose = false;
  bool has_raw_gps = false;
  bool has_gps_llh = false;
  bool has_gps_alt = false;

  mower_msgs::HighLevelStatus high_level;
  mower_msgs::Status low_level;
  mower_msgs::Power power;
  mower_msgs::Emergency emergency;
  mower_msgs::ESCStatus left_esc;
  mower_msgs::ESCStatus right_esc;
  xbot_msgs::AbsolutePose pose;
  xbot_msgs::AbsolutePose raw_gps;
  double gps_lat = 0.0;
  double gps_lon = 0.0;
  double gps_alt = 0.0;

  double high_level_seen = 0.0;
  double low_level_seen = 0.0;
  double power_seen = 0.0;
  double emergency_seen = 0.0;
  double left_esc_seen = 0.0;
  double right_esc_seen = 0.0;
  double pose_seen = 0.0;
  double raw_gps_seen = 0.0;
  double gps_llh_seen = 0.0;

  std::array<int, 3> sonar{{999, 999, 999}};
  bool bumper = false;
  bool bumper_left = false;
  bool bumper_right = false;
  bool rain_mega = false;
  bool tilt = false;
  bool wire_detected = true;
  double mega_compass_deg = 0.0;
  double mega_gyro_roll_deg = 0.0;
  double mega_gyro_pitch_deg = 0.0;
  double mega_gyro_yaw_deg = 0.0;
  double mega_gyro_rate_x = 0.0;
  double mega_gyro_rate_y = 0.0;
  double mega_gyro_rate_z = 0.0;
  std::unordered_map<std::string, bool> sensor_available;
  std::unordered_map<std::string, std::string> sensor_cause;
  std::unordered_map<std::string, std::string> mega_settings;
  bool mega_connected = false;
  std::string mega_connection_status = "Mega no conectado";
  std::string requested_mode = "idle";

  std::string map_json;
  double map_received_at = 0.0;

  std::optional<RecordingSession> recording;
  std::optional<BufferedRecording> last_recording_buffer;
  std::vector<std::vector<PointXY>> pending_obstacles;
};

}  // namespace

class MowerIosBridgeNode {
 public:
  MowerIosBridgeNode()
      : nh_(),
        pnh_("~"),
        host_(pnh_.param<std::string>("host", "0.0.0.0")),
        port_(pnh_.param<int>("port", 8080)),
        discovery_name_(pnh_.param<std::string>("discovery_name", "lawnmower")),
        auth_token_(pnh_.param<std::string>("auth_token", "")),
        linear_speed_(pnh_.param<double>("manual_linear_speed", 0.25)),
        angular_speed_(pnh_.param<double>("manual_angular_speed", 0.8)),
        manual_deadband_(pnh_.param<double>("manual_deadband", 0.08)),
        beacon_enabled_(rosBoolParam(pnh_, "udp_beacon_enabled", true)),
        beacon_port_(pnh_.param<int>("udp_beacon_port", 47820)),
        rec_min_distance_m_(pnh_.param<double>("rec_min_distance_m", 0.10)),
        rec_sample_hz_(pnh_.param<double>("rec_sample_hz", 4.0)),
        rec_close_tolerance_m_(pnh_.param<double>("rec_close_tolerance_m", 0.30)),
        rec_min_area_m2_(pnh_.param<double>("rec_min_area_m2", 0.50)),
        rec_require_rtk_fixed_(rosBoolParam(pnh_, "rec_require_rtk_fixed", true)),
        rec_fixed_accuracy_m_(pnh_.param<double>("rec_fixed_accuracy_m", 0.05)),
        rec_float_accuracy_m_(pnh_.param<double>("rec_float_accuracy_m", 0.50)),
        rec_fix_loss_pause_s_(pnh_.param<double>("rec_fix_loss_pause_s", 2.0)),
        map_update_timeout_s_(pnh_.param<double>("map_update_timeout_s", 3.0)),
        auto_schedule_enabled_(rosBoolParam(pnh_, "auto_schedule_enabled", false)) {
    pnh_.param<std::vector<std::string>>("auto_schedule_times", auto_schedule_times_, std::vector<std::string>{});
    pnh_.param<std::vector<int>>("auto_schedule_weekdays", auto_schedule_weekdays_, std::vector<int>{0, 1, 2, 3, 4, 5, 6});
    if (auto_schedule_times_.empty()) {
      const std::string csv = pnh_.param<std::string>("auto_schedule_times_csv", "");
      if (!csv.empty()) auto_schedule_times_ = splitCsv(csv);
    }
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      state_.started_at = wallNowSec();
    }

    action_pub_ = nh_.advertise<std_msgs::String>("/xbot/action", 5);
    joy_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    manual_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/ll/manual_cmd_vel", 1);
    blade_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/mega/blade_cmd", 1);
    cfgget_pub_ = nh_.advertise<std_msgs::Bool>("/mega/cfgget", 1);
    cfgset_pub_ = nh_.advertise<std_msgs::String>("/mega/cfgset", 10);
    rpc_request_pub_ = nh_.advertise<xbot_rpc::RpcRequest>("/xbot/rpc/request", 10);

    high_level_srv_ = nh_.serviceClient<mower_msgs::HighLevelControlSrv>("/mower_service/high_level_control");
    mower_control_srv_ = nh_.serviceClient<mower_msgs::MowerControlSrv>("/ll/_service/mow_enabled");
    emergency_stop_srv_ = nh_.serviceClient<mower_msgs::EmergencyStopSrv>("/ll/_service/emergency");
    add_area_srv_ = nh_.serviceClient<mower_map::AddMowingAreaSrv>("/mower_map_service/add_mowing_area");
    set_dock_srv_ = nh_.serviceClient<mower_map::SetDockingPointSrv>("/mower_map_service/set_docking_point");
    clear_map_srv_ = nh_.serviceClient<mower_map::ClearMapSrv>("/mower_map_service/clear_map");

    sub_high_level_ =
        nh_.subscribe("/mower_logic/current_state", 1, &MowerIosBridgeNode::cbHighLevel, this);
    sub_low_level_ =
        nh_.subscribe("/ll/mower_status", 1, &MowerIosBridgeNode::cbLowLevel, this);
    sub_power_ = nh_.subscribe("/ll/power", 1, &MowerIosBridgeNode::cbPower, this);
    sub_emergency_ =
        nh_.subscribe("/ll/emergency", 1, &MowerIosBridgeNode::cbEmergency, this);
    sub_left_esc_ = nh_.subscribe(
        "/ll/diff_drive/left_esc_status", 1, &MowerIosBridgeNode::cbLeftEsc, this);
    sub_right_esc_ = nh_.subscribe(
        "/ll/diff_drive/right_esc_status", 1, &MowerIosBridgeNode::cbRightEsc, this);
    sub_pose_ = nh_.subscribe("/xbot_positioning/xb_pose", 1, &MowerIosBridgeNode::cbPose, this);
    sub_raw_gps_ = nh_.subscribe("/ll/position/gps", 1, &MowerIosBridgeNode::cbRawGps, this);
    sub_gps_nmea_ =
        nh_.subscribe("/ll/position/gps/nmea", 3, &MowerIosBridgeNode::cbGpsNmea, this);

    sub_sonar_front_ = nh_.subscribe<sensor_msgs::Range>(
        "/mega/sonar/front", 1,
        [this](const sensor_msgs::Range::ConstPtr& msg) { cbSonar(msg, 0); });
    sub_sonar_left_ = nh_.subscribe<sensor_msgs::Range>(
        "/mega/sonar/left", 1,
        [this](const sensor_msgs::Range::ConstPtr& msg) { cbSonar(msg, 1); });
    sub_sonar_right_ = nh_.subscribe<sensor_msgs::Range>(
        "/mega/sonar/right", 1,
        [this](const sensor_msgs::Range::ConstPtr& msg) { cbSonar(msg, 2); });
    sub_bumper_ = nh_.subscribe("/mega/bumper", 1, &MowerIosBridgeNode::cbBumper, this);
    sub_bumper_left_ = nh_.subscribe("/mega/bumper_left", 1, &MowerIosBridgeNode::cbBumperLeft, this);
    sub_bumper_right_ = nh_.subscribe("/mega/bumper_right", 1, &MowerIosBridgeNode::cbBumperRight, this);
    sub_rain_ = nh_.subscribe("/mega/rain", 1, &MowerIosBridgeNode::cbRainMega, this);
    sub_tilt_ = nh_.subscribe("/mega/tilt", 1, &MowerIosBridgeNode::cbTilt, this);
    sub_wire_ =
        nh_.subscribe("/mega/wire_detected", 1, &MowerIosBridgeNode::cbWireDetected, this);
    sub_compass_imu_ = nh_.subscribe("/mega/imu", 1, &MowerIosBridgeNode::cbCompassImu, this);
    sub_gyro_imu_ = nh_.subscribe("/mega/imu_gyro", 1, &MowerIosBridgeNode::cbGyroImu, this);
    sub_cfg_ = nh_.subscribe("/mega/cfg", 10, &MowerIosBridgeNode::cbCfg, this);
    sub_cfg_loaded_ =
        nh_.subscribe("/mega/cfg_loaded", 1, &MowerIosBridgeNode::cbCfgLoaded, this);
    sub_sstat_ = nh_.subscribe("/mega/sstat", 10, &MowerIosBridgeNode::cbSStat, this);
    sub_mega_connected_ =
        nh_.subscribe("/mega/connected", 1, &MowerIosBridgeNode::cbMegaConnected, this);
    sub_mega_connection_status_ = nh_.subscribe(
        "/mega/connection_status", 1, &MowerIosBridgeNode::cbMegaConnectionStatus, this);
    sub_json_map_ =
        nh_.subscribe("/mower_map_service/json_map", 1, &MowerIosBridgeNode::cbJsonMap, this);
    sub_rpc_response_ =
        nh_.subscribe("/xbot/rpc/response", 10, &MowerIosBridgeNode::cbRpcResponse, this);
    sub_rpc_error_ = nh_.subscribe("/xbot/rpc/error", 10, &MowerIosBridgeNode::cbRpcError, this);

    const double sample_period = std::max(0.05, 1.0 / std::max(0.5, rec_sample_hz_));
    recording_timer_ =
        nh_.createTimer(ros::Duration(sample_period), &MowerIosBridgeNode::cbSampleRecording, this);
    settings_timer_ =
        nh_.createTimer(ros::Duration(3.0), &MowerIosBridgeNode::cbRequestSettings, this, true, true);
    schedule_timer_ =
        nh_.createTimer(ros::Duration(1.0), &MowerIosBridgeNode::cbAutoSchedule, this, false, true);
  }

  ~MowerIosBridgeNode() {
    stop();
  }

  void start() {
    if (running_.exchange(true)) return;
    startHttpServer();
    if (beacon_enabled_.load()) {
      beacon_thread_ = std::thread(&MowerIosBridgeNode::udpBeaconLoop, this);
    }
  }

  void stop() {
    if (!running_.exchange(false)) return;
    boost::system::error_code ec;
    if (acceptor_) {
      acceptor_->close(ec);
    }
    io_context_.stop();
    if (http_thread_.joinable()) http_thread_.join();
    if (beacon_thread_.joinable()) beacon_thread_.join();
  }

 private:
  // ROS and shared state
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  mutable std::recursive_mutex state_mutex_;
  std::condition_variable_any state_cv_;
  StateSnapshot state_;
  std::recursive_mutex service_mutex_;
  std::mutex rpc_pending_mutex_;
  std::unordered_map<std::string, std::shared_ptr<RpcPending>> rpc_pending_;
  std::atomic<std::uint64_t> rpc_seq_{0};

  // Params / runtime config
  std::string host_;
  int port_;
  std::string discovery_name_;
  std::string auth_token_;
  std::atomic<double> linear_speed_;
  std::atomic<double> angular_speed_;
  std::atomic<double> manual_deadband_;
  std::atomic<bool> beacon_enabled_;
  int beacon_port_;
  double rec_min_distance_m_;
  double rec_sample_hz_;
  double rec_close_tolerance_m_;
  double rec_min_area_m2_;
  bool rec_require_rtk_fixed_;
  double rec_fixed_accuracy_m_;
  double rec_float_accuracy_m_;
  double rec_fix_loss_pause_s_;
  double map_update_timeout_s_;
  bool auto_schedule_enabled_;
  std::vector<std::string> auto_schedule_times_;
  std::vector<int> auto_schedule_weekdays_;
  std::string last_schedule_fire_key_;
  std::atomic<double> last_app_seen_at_{0.0};

  // ROS pubs/subs/services
  ros::Publisher action_pub_;
  ros::Publisher joy_vel_pub_;
  ros::Publisher manual_cmd_vel_pub_;
  ros::Publisher blade_cmd_pub_;
  ros::Publisher cfgget_pub_;
  ros::Publisher cfgset_pub_;
  ros::Publisher rpc_request_pub_;

  ros::ServiceClient high_level_srv_;
  ros::ServiceClient mower_control_srv_;
  ros::ServiceClient emergency_stop_srv_;
  ros::ServiceClient add_area_srv_;
  ros::ServiceClient set_dock_srv_;
  ros::ServiceClient clear_map_srv_;

  ros::Subscriber sub_high_level_;
  ros::Subscriber sub_low_level_;
  ros::Subscriber sub_power_;
  ros::Subscriber sub_emergency_;
  ros::Subscriber sub_left_esc_;
  ros::Subscriber sub_right_esc_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_raw_gps_;
  ros::Subscriber sub_gps_nmea_;
  ros::Subscriber sub_sonar_front_;
  ros::Subscriber sub_sonar_left_;
  ros::Subscriber sub_sonar_right_;
  ros::Subscriber sub_bumper_;
  ros::Subscriber sub_bumper_left_;
  ros::Subscriber sub_bumper_right_;
  ros::Subscriber sub_rain_;
  ros::Subscriber sub_tilt_;
  ros::Subscriber sub_wire_;
  ros::Subscriber sub_compass_imu_;
  ros::Subscriber sub_gyro_imu_;
  ros::Subscriber sub_cfg_;
  ros::Subscriber sub_cfg_loaded_;
  ros::Subscriber sub_sstat_;
  ros::Subscriber sub_mega_connected_;
  ros::Subscriber sub_mega_connection_status_;
  ros::Subscriber sub_json_map_;
  ros::Subscriber sub_rpc_response_;
  ros::Subscriber sub_rpc_error_;

  ros::Timer recording_timer_;
  ros::Timer settings_timer_;
  ros::Timer schedule_timer_;

  // HTTP / UDP
  std::atomic<bool> running_{false};
  asio::io_context io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread http_thread_;
  std::thread beacon_thread_;

  // ----- ROS callbacks -----

  void cbHighLevel(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.high_level = *msg;
    state_.has_high_level = true;
    state_.high_level_seen = wallNowSec();
  }

  void cbLowLevel(const mower_msgs::Status::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.low_level = *msg;
    state_.has_low_level = true;
    state_.low_level_seen = wallNowSec();
  }

  void cbPower(const mower_msgs::Power::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.power = *msg;
    state_.has_power = true;
    state_.power_seen = wallNowSec();
  }

  void cbEmergency(const mower_msgs::Emergency::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.emergency = *msg;
    state_.has_emergency = true;
    state_.emergency_seen = wallNowSec();
  }

  void cbLeftEsc(const mower_msgs::ESCStatus::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.left_esc = *msg;
    state_.has_left_esc = true;
    state_.left_esc_seen = wallNowSec();
  }

  void cbRightEsc(const mower_msgs::ESCStatus::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.right_esc = *msg;
    state_.has_right_esc = true;
    state_.right_esc_seen = wallNowSec();
  }

  void cbPose(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.pose = *msg;
    state_.has_pose = true;
    state_.pose_seen = wallNowSec();
  }

  void cbRawGps(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.raw_gps = *msg;
    state_.has_raw_gps = true;
    state_.raw_gps_seen = wallNowSec();
  }

  static std::vector<std::string> splitNmeaSentence(const std::string& sentence) {
    std::vector<std::string> fields;
    std::string current;
    for (char c : sentence) {
      if (c == ',') {
        fields.push_back(current);
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    fields.push_back(current);
    return fields;
  }

  static std::optional<double> parseNmeaCoordinate(const std::string& raw, const std::string& hemi, bool lat) {
    if (raw.empty() || hemi.empty()) return std::nullopt;
    int deg_digits = lat ? 2 : 3;
    if (static_cast<int>(raw.size()) <= deg_digits) return std::nullopt;
    try {
      const double deg = std::stod(raw.substr(0, static_cast<std::size_t>(deg_digits)));
      const double minutes = std::stod(raw.substr(static_cast<std::size_t>(deg_digits)));
      double value = deg + minutes / 60.0;
      const char h = static_cast<char>(std::toupper(static_cast<unsigned char>(hemi[0])));
      if (h == 'S' || h == 'W') value = -value;
      return value;
    } catch (...) {
      return std::nullopt;
    }
  }

  void cbGpsNmea(const nmea_msgs::Sentence::ConstPtr& msg) {
    const std::string& s = msg->sentence;
    if (s.empty() || s[0] != '$') return;
    const std::size_t star = s.find('*');
    const std::string payload = s.substr(0, star);
    const std::vector<std::string> fields = splitNmeaSentence(payload);
    if (fields.empty()) return;
    const std::string& head = fields[0];
    bool parsed = false;
    std::optional<double> lat;
    std::optional<double> lon;
    std::optional<double> alt;

    if (head.size() >= 6 && head.substr(head.size() - 3) == "GGA" && fields.size() > 9) {
      lat = parseNmeaCoordinate(fields[2], fields[3], true);
      lon = parseNmeaCoordinate(fields[4], fields[5], false);
      if (!fields[9].empty()) {
        try {
          alt = std::stod(fields[9]);
        } catch (...) {
        }
      }
      parsed = true;
    } else if (head.size() >= 6 && head.substr(head.size() - 3) == "RMC" && fields.size() > 6) {
      lat = parseNmeaCoordinate(fields[3], fields[4], true);
      lon = parseNmeaCoordinate(fields[5], fields[6], false);
      parsed = true;
    }

    if (!parsed || !lat.has_value() || !lon.has_value()) return;
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.gps_lat = *lat;
    state_.gps_lon = *lon;
    if (alt.has_value()) {
      state_.gps_alt = *alt;
      state_.has_gps_alt = true;
    }
    state_.has_gps_llh = true;
    state_.gps_llh_seen = wallNowSec();
  }

  void cbSonar(const sensor_msgs::Range::ConstPtr& msg, int idx) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (idx < 0 || idx >= 3) return;
    if (!std::isfinite(msg->range) || msg->range >= 9.99) {
      state_.sonar[static_cast<std::size_t>(idx)] = 999;
    } else {
      state_.sonar[static_cast<std::size_t>(idx)] = static_cast<int>(msg->range * 100.0);
    }
  }

  void cbBumper(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.bumper = msg->data;
  }

  void cbBumperLeft(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.bumper_left = msg->data;
    state_.bumper = state_.bumper_left || state_.bumper_right;
  }

  void cbBumperRight(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.bumper_right = msg->data;
    state_.bumper = state_.bumper_left || state_.bumper_right;
  }

  void cbRainMega(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.rain_mega = msg->data;
  }

  void cbTilt(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.tilt = msg->data;
  }

  void cbWireDetected(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.wire_detected = msg->data;
  }

  void cbCompassImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& q = msg->orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double deg = yaw * 180.0 / M_PI;
    if (deg < 0.0) deg += 360.0;
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.mega_compass_deg = deg;
  }

  void cbGyroImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto& q = msg->orientation;
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.mega_gyro_roll_deg = roll * 180.0 / M_PI;
    state_.mega_gyro_pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg = yaw * 180.0 / M_PI;
    if (yaw_deg < 0.0) yaw_deg += 360.0;
    state_.mega_gyro_yaw_deg = yaw_deg;
    state_.mega_gyro_rate_x = msg->angular_velocity.x;
    state_.mega_gyro_rate_y = msg->angular_velocity.y;
    state_.mega_gyro_rate_z = msg->angular_velocity.z;
  }

  void cbMegaConnected(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.mega_connected = msg->data;
  }

  void cbMegaConnectionStatus(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.mega_connection_status =
        msg->data.empty() ? std::string("Mega no conectado") : msg->data;
  }

  void cbCfg(const std_msgs::String::ConstPtr& msg) {
    const auto eq = msg->data.find('=');
    if (eq == std::string::npos) return;
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.mega_settings[msg->data.substr(0, eq)] = msg->data.substr(eq + 1);
    state_cv_.notify_all();
  }

  void cbCfgLoaded(const std_msgs::Bool::ConstPtr&) {
    StateSnapshot snap = snapshot();
    ROS_INFO("[ios_bridge] Mega settings loaded (%zu keys)", snap.mega_settings.size());
  }

  void cbSStat(const std_msgs::String::ConstPtr& msg) {
    // format: sensor,state,cause
    std::string s = msg->data;
    size_t a = s.find(',');
    if (a == std::string::npos) return;
    size_t b = s.find(',', a + 1);
    if (b == std::string::npos) return;
    std::string sensor = s.substr(0, a);
    std::string state = s.substr(a + 1, b - a - 1);
    std::string cause = s.substr(b + 1);
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.sensor_available[sensor] = (state == "OK");
    state_.sensor_cause[sensor] = cause;
  }

  void cbJsonMap(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.map_json = msg->data;
    state_.map_received_at = wallNowSec();
    state_cv_.notify_all();
  }

  void cbRpcResponse(const xbot_rpc::RpcResponse::ConstPtr& msg) {
    std::shared_ptr<RpcPending> pending;
    {
      std::lock_guard<std::mutex> lock(rpc_pending_mutex_);
      auto it = rpc_pending_.find(msg->id);
      if (it == rpc_pending_.end()) return;
      pending = it->second;
    }
    {
      std::lock_guard<std::mutex> lock(pending->mutex);
      pending->done = true;
      pending->error = false;
      pending->result = msg->result;
    }
    pending->cv.notify_all();
  }

  void cbRpcError(const xbot_rpc::RpcError::ConstPtr& msg) {
    std::shared_ptr<RpcPending> pending;
    {
      std::lock_guard<std::mutex> lock(rpc_pending_mutex_);
      auto it = rpc_pending_.find(msg->id);
      if (it == rpc_pending_.end()) return;
      pending = it->second;
    }
    {
      std::lock_guard<std::mutex> lock(pending->mutex);
      pending->done = true;
      pending->error = true;
      pending->code = msg->code;
      pending->message = msg->message;
    }
    pending->cv.notify_all();
  }

  void cbSampleRecording(const ros::TimerEvent&) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (!state_.recording.has_value()) return;

    const xbot_msgs::AbsolutePose* sample_pose = nullptr;
    if (state_.has_pose) {
      sample_pose = &state_.pose;
    } else if (state_.has_raw_gps) {
      sample_pose = &state_.raw_gps;
    }
    if (sample_pose == nullptr) return;

    RecordingSession& rec = *state_.recording;
    if (!recordingAllowedLocked()) {
      rec.paused = true;
      rec.pause_reason = "robot_state";
      if (!rec.has_paused_at) {
        rec.paused_at = wallNowSec();
        rec.has_paused_at = true;
      }
      return;
    }

    const std::string fix = gpsFixType(*sample_pose);
    rec.last_fix = fix;
    if (rec_require_rtk_fixed_ && fix != "fixed") {
      const double now = wallNowSec();
      if (!rec.has_bad_fix_since) {
        rec.bad_fix_since = now;
        rec.has_bad_fix_since = true;
      } else if ((now - rec.bad_fix_since) >= rec_fix_loss_pause_s_) {
        rec.paused = true;
        rec.pause_reason = "rtk_fix_lost";
        if (!rec.has_paused_at) {
          rec.paused_at = now;
          rec.has_paused_at = true;
        }
      }
      rec.rtk_lost_count += 1;
      return;
    }

    rec.has_bad_fix_since = false;
    if (rec.paused) return;

    const double x = static_cast<double>(sample_pose->pose.pose.position.x);
    const double y = static_cast<double>(sample_pose->pose.pose.position.y);
    if (!rec.points.empty()) {
      const PointXY& last = rec.points.back();
      if (std::hypot(x - last.x, y - last.y) < rec_min_distance_m_) return;
    }
    rec.points.push_back({x, y});
    rec.sampled_at = wallNowSec();
  }

  void cbRequestSettings(const ros::TimerEvent&) {
    std_msgs::Bool msg;
    msg.data = true;
    cfgget_pub_.publish(msg);
    ROS_INFO("[ios_bridge] requested settings from Mega");
  }

  bool scheduleWeekdayAllowed(int weekday) const {
    for (int d : auto_schedule_weekdays_) {
      if (d == weekday) return true;
    }
    return false;
  }

  bool parseHourMinute(const std::string& value, int& hour, int& minute) const {
    if (value.size() != 5 || value[2] != ':') return false;
    if (!std::isdigit(static_cast<unsigned char>(value[0])) ||
        !std::isdigit(static_cast<unsigned char>(value[1])) ||
        !std::isdigit(static_cast<unsigned char>(value[3])) ||
        !std::isdigit(static_cast<unsigned char>(value[4]))) {
      return false;
    }
    hour = std::stoi(value.substr(0, 2));
    minute = std::stoi(value.substr(3, 2));
    return hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59;
  }

  void cbAutoSchedule(const ros::TimerEvent&) {
    if (!auto_schedule_enabled_ || auto_schedule_times_.empty()) return;

    const StateSnapshot snap = snapshot();
    if (!snap.mega_connected) return;
    if (snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency)) return;
    if (controlMode(snap) != "dock") return;

    const std::time_t now_t = std::time(nullptr);
    std::tm now_tm{};
#if defined(_WIN32)
    localtime_s(&now_tm, &now_t);
#else
    localtime_r(&now_t, &now_tm);
#endif
    // tm_wday: 0=Sunday..6=Saturday -> 0=Monday..6=Sunday
    const int weekday = (now_tm.tm_wday + 6) % 7;
    if (!scheduleWeekdayAllowed(weekday)) return;

    for (const std::string& hhmm : auto_schedule_times_) {
      int h = 0;
      int m = 0;
      if (!parseHourMinute(hhmm, h, m)) continue;
      if (h != now_tm.tm_hour || m != now_tm.tm_min) continue;

      char daybuf[16];
      std::snprintf(daybuf, sizeof(daybuf), "%04d-%02d-%02d", now_tm.tm_year + 1900, now_tm.tm_mon + 1, now_tm.tm_mday);
      const std::string key = std::string(daybuf) + "-" + hhmm;
      if (key == last_schedule_fire_key_) return;

      try {
        setRequestedMode("auto");
        checkHardware(snap, true);
        callHighLevel(mower_msgs::HighLevelControlSrvRequest::COMMAND_START);
        last_schedule_fire_key_ = key;
        ROS_INFO("[ios_bridge] auto schedule fired at %s", hhmm.c_str());
      } catch (const std::exception& exc) {
        ROS_WARN("[ios_bridge] auto schedule failed at %s: %s", hhmm.c_str(), exc.what());
      }
      return;
    }
  }

  void touchAppSession() {
    const double now = wallNowSec();
    const double last = last_app_seen_at_.load();
    if ((now - last) > 5.0) {
      std_msgs::Bool msg;
      msg.data = true;
      cfgget_pub_.publish(msg);
      ROS_INFO("[ios_bridge] app session active: requesting Mega settings/telemetry");
    }
    last_app_seen_at_.store(now);
  }

  // ----- State helpers -----

  bool manualActiveLocked() {
    if (state_.manual_active && wallNowSec() > state_.manual_until) {
      state_.manual_active = false;
    }
    return state_.manual_active;
  }

  void setManualActive(bool active, double hold_seconds = 2.0) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.manual_active = active;
    state_.manual_until = active ? (wallNowSec() + hold_seconds) : 0.0;
  }

  StateSnapshot snapshot() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    manualActiveLocked();
    return state_;
  }

  bool recordingAllowedLocked() {
    if (state_.has_emergency && state_.emergency.latched_emergency) return false;
    if (manualActiveLocked()) return true;
    if (!state_.has_high_level) return false;
    return (state_.high_level.state & 0b11111) == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
  }

  // ----- JSON payloads -----

  json healthPayload() {
    const StateSnapshot snap = snapshot();
    return json{
        {"ok", true},
        {"uptimeMs", static_cast<std::int64_t>((wallNowSec() - snap.started_at) * 1000.0)},
        {"ip", localIp()},
        {"manualActive", snap.manual_active},
        {"mode", controlMode(snap)},
    };
  }

  bool isAutoRunningState(const std::string& state) const {
    return state == "mowing" || state == "trackingWire" || state == "exitingDock";
  }

  std::string controlMode(const StateSnapshot& snap) const {
    const bool charging =
        (snap.has_low_level && snap.low_level.is_charging) || (snap.has_power && snap.power.charge_current > 0.1f);
    const std::string op_state = operatingState(snap);
    if (snap.manual_active || op_state == "manual") return "manual";
    if (charging && (op_state == "docked" || op_state == "parked" || op_state == "unknown")) return "dock";
    if (isAutoRunningState(op_state)) return "auto";
    return "idle";
  }

  void setRequestedMode(const std::string& mode) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.requested_mode = mode;
  }

  std::string operatingState(const StateSnapshot& snap) const {
    const bool emergency_active =
        snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency);
    const bool charging =
        (snap.has_low_level && snap.low_level.is_charging) || (snap.has_power && snap.power.charge_current > 0.1f);
    if (emergency_active) return "error";
    if (snap.manual_active) return "manual";
    if (!snap.has_high_level) return charging ? "docked" : "unknown";

    std::string name = snap.high_level.state_name;
    std::string sub = snap.high_level.sub_state_name;
    for (char& c : name) c = static_cast<char>(std::toupper(c));
    for (char& c : sub) c = static_cast<char>(std::toupper(c));
    const std::string text = name + " " + sub;
    const int state_base = snap.high_level.state & 0b11111;

    if (text.find("UNDOCK") != std::string::npos) return "exitingDock";
    if (text.find("DOCK") != std::string::npos && !charging) return "trackingWire";
    if (text.find("MOW") != std::string::npos ||
        state_base == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS) {
      return "mowing";
    }
    if (state_base == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING) return "setup";
    if (state_base == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE) {
      return charging || (snap.has_low_level && snap.low_level.is_charging) ? "docked" : "parked";
    }
    return "unknown";
  }

  std::string emergencyReasonText(const StateSnapshot& snap) const {
    if (snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency)) {
      if (!snap.emergency.reason.empty()) return snap.emergency.reason;
      return "emergency_active";
    }
    return "";
  }

  std::string degradationReasonText(const StateSnapshot& snap) const {
    const double now = wallNowSec();
    constexpr double pose_timeout_s = 1.0;
    constexpr double raw_gps_timeout_s = 10.0;
    const bool raw_gps_available = snap.has_raw_gps && (now - snap.raw_gps_seen) <= raw_gps_timeout_s;
    if (!snap.has_pose) return raw_gps_available ? "pose_fusion_missing" : "pose_missing";
    if ((now - snap.pose_seen) > pose_timeout_s) return raw_gps_available ? "pose_fusion_timeout" : "pose_timeout";
    return "";
  }

  json statusPayload() {
    const StateSnapshot snap = snapshot();
    const bool emergency_active =
        snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency);
    const bool charging =
        (snap.has_low_level && snap.low_level.is_charging) || (snap.has_power && snap.power.charge_current > 0.1f);
    const double battery_voltage = firstValid({
        snap.has_power ? static_cast<double>(snap.power.battery_voltage_adc) : 0.0,
        snap.has_power ? static_cast<double>(snap.power.battery_voltage_bms) : 0.0,
        snap.has_power ? static_cast<double>(snap.power.battery_voltage_chg) : 0.0,
    });

    double wheel_current = 0.0;
    if (snap.has_left_esc) wheel_current += std::abs(static_cast<double>(snap.left_esc.current));
    if (snap.has_right_esc) wheel_current += std::abs(static_cast<double>(snap.right_esc.current));

    const std::string state = operatingState(snap);
    const std::string mode = controlMode(snap);
    const bool running =
        state == "mowing" || state == "trackingWire" || state == "exitingDock" || state == "manual";
    const bool docked = state == "docked";
    const bool parked = state == "parked";
    const bool rain_detected =
        (snap.has_low_level && snap.low_level.rain_detected) || snap.rain_mega;

    const std::string gps_fix_type = gpsFixTypeOrNone(snap);
    const std::string gps_fix_label = gpsFixLabel(gps_fix_type);
    const bool has_ui_pose = uiPoseSource(snap) != nullptr;
    const bool gps_llh_fresh = snap.has_gps_llh && (wallNowSec() - snap.gps_llh_seen) <= 10.0;
    const std::string emergency_reason = emergencyReasonText(snap);
    const std::string degradation_reason = degradationReasonText(snap);

    return json{
        {"connection", snap.mega_connected ? "connected" : "disconnected"},
        {"connectionMessage", snap.mega_connection_status},
        {"state", state},
        {"mode", mode},
        {"requestedMode", snap.requested_mode},
        {"robotStatus", snap.has_low_level ? static_cast<int>(snap.low_level.mower_status) : 0},
        {"errorCode", emergency_active ? 3 : (snap.tilt ? 4 : 0)},
        {"batteryVoltage", battery_voltage},
        {"chargeCurrent", snap.has_power ? static_cast<double>(snap.power.charge_current) : 0.0},
        {"wheelCurrent", wheel_current},
        {"charging", charging},
        {"docked", docked},
        {"parked", parked},
        {"running", running},
        {"trackingWire", state == "trackingWire"},
        {"wireDetected", snap.wire_detected},
        {"outsideWire", !snap.wire_detected && running},
        {"rainDetected", rain_detected},
        {"bladesOn", snap.has_low_level && snap.low_level.mow_enabled},
        {"bumper", snap.bumper},
        {"bumperLeft", snap.bumper_left},
        {"bumperRight", snap.bumper_right},
        {"tiltActive", snap.tilt},
        {"lowBattery", false},
        {"wheelBlocked", false},
        {"alarm1Enabled", snap.tilt},
        {"alarm2Enabled", rain_detected},
        {"alarm3Enabled", false},
        {"gpsHasPose", has_ui_pose},
        {"gpsFixType", gps_fix_type},
        {"gpsFixLabel", gps_fix_label},
        {"gpsPositionAccuracy", snap.has_pose ? static_cast<double>(snap.pose.position_accuracy) : -1.0},
        {"emergencyActive", emergency_active},
        {"emergencyLatched", snap.has_emergency ? static_cast<bool>(snap.emergency.latched_emergency) : false},
        {"emergencyReason", emergency_reason},
        {"degradationReason", degradation_reason},
        {"gpsLat", gps_llh_fresh ? json(snap.gps_lat) : json(nullptr)},
        {"gpsLon", gps_llh_fresh ? json(snap.gps_lon) : json(nullptr)},
        {"lastUpdated", nowMs()},
    };
  }

  json telemetryPayload() {
    const StateSnapshot snap = snapshot();
    double heading = 0.0;
    double compass_error = 0.0;
    if (snap.has_pose) {
      heading = snap.pose.vehicle_heading * 180.0 / M_PI;
      compass_error = static_cast<double>(snap.pose.position_accuracy);
    }
    const int pwm_left = snap.has_left_esc ? rpmToPwm(snap.left_esc.rpm) : 0;
    const int pwm_right = snap.has_right_esc ? rpmToPwm(snap.right_esc.rpm) : 0;
    int wheel_status = 7;
    if (pwm_left > 0 || pwm_right > 0) wheel_status = 5;
    const bool sonar_triggered =
        (snap.sonar[0] < 999 && snap.sonar[0] < 30) ||
        (snap.sonar[1] < 999 && snap.sonar[1] < 30) ||
        (snap.sonar[2] < 999 && snap.sonar[2] < 30);

    json sensorAvail = json::object();
    json sensorCause = json::object();
    // Copy explicit values from /mega/sensor_status topic
    for (const auto& kv : snap.sensor_available) sensorAvail[kv.first] = kv.second;
    for (const auto& kv : snap.sensor_cause) sensorCause[kv.first] = kv.second;

    // Derive availability from last_seen timestamps and mega settings
    const double now = wallNowSec();
    constexpr double stale_s = 10.0;
    const bool emergency_active =
        snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency);
    const std::string emergency_reason = emergencyReasonText(snap);
    const std::string degradation_reason = degradationReasonText(snap);

    auto megaOn = [&](const std::string& key) -> int {
      // returns 1=on, 0=off, -1=unknown
      auto it = snap.mega_settings.find(key);
      if (it == snap.mega_settings.end()) return -1;
      try { return std::stof(it->second) >= 0.5f ? 1 : 0; }
      catch (...) { return -1; }
    };

    auto markUnavailable = [&](const std::string& sensor, const std::string& cause) {
      if (sensorAvail.find(sensor) == sensorAvail.end()) {
        sensorAvail[sensor] = false;
        sensorCause[sensor] = cause;
      }
    };

    // Battery: power topic must be alive
    if (!snap.has_power || (now - snap.power_seen) > stale_s)
      markUnavailable("BATT", "NO_CONN");

    // GPS: consider either fused pose or raw GPS stream
    const bool pose_fresh = snap.has_pose && (now - snap.pose_seen) <= stale_s;
    const bool raw_gps_fresh = snap.has_raw_gps && (now - snap.raw_gps_seen) <= stale_s;
    if (!pose_fresh && !raw_gps_fresh)
      markUnavailable("GPS", "NO_CONN");

    // Compass
    if (megaOn("compassOn") == 0)
      markUnavailable("COMPASS", "DISABLED");
    else if (megaOn("compassOn") == 1 && !snap.mega_connected)
      markUnavailable("COMPASS", "NO_CONN");

    // Gyroscope
    if (megaOn("gyroOn") == 0)
      markUnavailable("GYRO", "DISABLED");

    // Sonar
    if (megaOn("sonar1On") == 0 && megaOn("sonar2On") == 0 && megaOn("sonar3On") == 0)
      markUnavailable("SONAR", "DISABLED");
    else if (snap.sonar[0] >= 999 && snap.sonar[1] >= 999 && snap.sonar[2] >= 999 && !snap.mega_connected)
      markUnavailable("SONAR", "NO_CONN");

    // Bumper
    if (megaOn("bumperOn") == 0)
      markUnavailable("BUMPER", "DISABLED");

    // Rain
    if (megaOn("rainOn") == 0)
      markUnavailable("RAIN", "DISABLED");

    // Tilt
    if (megaOn("tiltOn") == 0)
      markUnavailable("TILT", "DISABLED");

    // Wire
    if (megaOn("wireOn") == 0)
      markUnavailable("WIRE", "DISABLED");

    // Wheel current
    if (megaOn("wheelAmpOn") == 0)
      markUnavailable("WHEEL_AMPS", "DISABLED");

    // Blade
    if (megaOn("bladeOn") == 0)
      markUnavailable("BLADE", "DISABLED");

    // Charge station
    if (megaOn("chargeStOn") == 0)
      markUnavailable("CHARGE", "DISABLED");

    return json{
        {"loopCycle", static_cast<int>(std::time(nullptr) % 100000)},
        {"sonarCenterCm", snap.sonar[0]},
        {"sonarLeftCm", snap.sonar[1]},
        {"sonarRightCm", snap.sonar[2]},
        {"sonarLeftHits", 0},
        {"sonarCenterHits", 0},
        {"sonarRightHits", 0},
        {"sonarTriggered", sonar_triggered},
        {"bumper", snap.bumper},
        {"bumperLeft", snap.bumper_left},
        {"bumperRight", snap.bumper_right},
        {"tiltAngle", snap.tilt},
        {"tipOver", snap.tilt},
        {"gpsInsideFence", gpsFixTypeOrNone(snap) != "none"},
        {"gpsFixType", gpsFixTypeOrNone(snap)},
        {"emergencyActive", emergency_active},
        {"emergencyLatched", snap.has_emergency ? static_cast<bool>(snap.emergency.latched_emergency) : false},
        {"emergencyReason", emergency_reason},
        {"degradationReason", degradation_reason},
        {"compassHeading", heading},
        {"compassError", compass_error},
        {"megaCompassDeg", snap.mega_compass_deg},
        {"megaGyroRollDeg", snap.mega_gyro_roll_deg},
        {"megaGyroPitchDeg", snap.mega_gyro_pitch_deg},
        {"megaGyroYawDeg", snap.mega_gyro_yaw_deg},
        {"megaGyroRateX", snap.mega_gyro_rate_x},
        {"megaGyroRateY", snap.mega_gyro_rate_y},
        {"megaGyroRateZ", snap.mega_gyro_rate_z},
        {"sensorAvailable", sensorAvail},
        {"sensorCause", sensorCause},
        {"magNow", 0},
        {"pwmLeft", pwm_left},
        {"pwmRight", pwm_right},
        {"mowerRunBack", 0},
        {"turnPhase", 0},
        {"sonarPhase", 0},
        {"wheelStatusValue", wheel_status},
    };
  }

  json settingsPayload() {
    const StateSnapshot snap = snapshot();
    auto mv = [&](const std::string& key, double default_value) -> double {
      auto it = snap.mega_settings.find(key);
      if (it != snap.mega_settings.end()) {
        try {
          return std::stod(it->second);
        } catch (...) {
        }
      }
      return default_value;
    };

    json settings = json::array();
    auto add = [&](const std::string& id, const std::string& title, const std::string& group,
                   const std::string& kind, double value, const char* unit,
                   std::optional<double> minimum, std::optional<double> maximum,
                   std::optional<double> step, std::optional<std::vector<std::string>> option_labels = std::nullopt) {
      json item{
          {"id", id},
          {"title", title},
          {"group", group},
          {"kind", kind},
          {"value", value},
          {"unit", unit ? json(unit) : json(nullptr)},
          {"minimum", minimum ? json(*minimum) : json(nullptr)},
          {"maximum", maximum ? json(*maximum) : json(nullptr)},
          {"step", step ? json(*step) : json(nullptr)},
          {"isEnabled", nullptr},
      };
      if (option_labels) item["optionLabels"] = *option_labels;
      settings.push_back(item);
    };

    add("bridge.manualLinearSpeed", "Velocidad lineal manual", "App iOS", "number",
        linear_speed_.load(), "m/s", 0.05, 1.0, 0.05);
    add("bridge.manualAngularSpeed", "Velocidad giro manual", "App iOS", "number",
        angular_speed_.load(), "rad/s", 0.1, 2.0, 0.1);
    add("bridge.udpBeacon", "Descubrimiento UDP", "App iOS", "boolean",
        beacon_enabled_.load() ? 1.0 : 0.0, nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.pwmMaxLH", "PWM máx izquierdo", "Motores de rueda", "number", mv("pwmMaxLH", 200), nullptr, 0, 255, 1);
    add("mega.pwmMaxRH", "PWM máx derecho", "Motores de rueda", "number", mv("pwmMaxRH", 200), nullptr, 0, 255, 1);
    add("mega.pwmSlowLH", "PWM lento izquierdo", "Motores de rueda", "number", mv("pwmSlowLH", 80), nullptr, 0, 255, 1);
    add("mega.pwmSlowRH", "PWM lento derecho", "Motores de rueda", "number", mv("pwmSlowRH", 80), nullptr, 0, 255, 1);
    add("mega.wheelsOn", "Ruedas activadas", "Motores de rueda", "boolean", mv("wheelsOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.turnDelayMin", "Tiempo giro mín (×100 ms)", "Temporización", "number", mv("turnDelayMin", 10), "×100ms", 1, 50, 1);
    add("mega.turnDelayMax", "Tiempo giro máx (×100 ms)", "Temporización", "number", mv("turnDelayMax", 20), "×100ms", 1, 100, 1);
    add("mega.reverseDelay", "Tiempo marcha atrás", "Temporización", "number", mv("reverseDelay", 5), "×100ms", 1, 50, 1);
    add("mega.straightCycles", "Ciclos en línea recta", "Temporización", "number", mv("straightCycles", 20), "ciclos", 1, 200, 1);
    add("mega.turn90LH", "Giro 90° izq (×10 ms)", "Temporización", "number", mv("turn90LH", 90), "×10ms", 1, 200, 1);
    add("mega.turn90RH", "Giro 90° der (×10 ms)", "Temporización", "number", mv("turn90RH", 90), "×10ms", 1, 200, 1);
    add("mega.lineLenCycles", "Ciclos longitud línea", "Temporización", "number", mv("lineLenCycles", 0), "ciclos", 0, 200, 1);

    add("mega.pwmBlade", "PWM cuchilla", "Cuchillas", "number", mv("pwmBlade", 255), nullptr, 0, 255, 1);
    add("mega.bladeOn", "Cuchillas activadas", "Cuchillas", "boolean", mv("bladeOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.sonar1On", "Sonar 1 activo", "Sonares", "boolean", mv("sonar1On", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.sonar2On", "Sonar 2 activo", "Sonares", "boolean", mv("sonar2On", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.sonar3On", "Sonar 3 activo", "Sonares", "boolean", mv("sonar3On", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.sonarMaxCm", "Distancia detección (cm)", "Sonares", "number", mv("sonarMaxCm", 30), "cm", 5, 200, 1);
    add("mega.sonarMaxHit", "Sensibilidad sonar", "Sonares", "number", mv("sonarMaxHit", 3), "hits", 1, 20, 1);

    add("mega.wireOn", "Cable perimetral activo", "Cable perimetral", "boolean", mv("wireOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.pidP", "PID P", "Cable perimetral", "number", mv("pidP", 1.50), nullptr, 0.01, 10.0, 0.01);
    add("mega.wireZ1Cyc", "Ciclos zona 1 (×100)", "Cable perimetral", "number", mv("wireZ1Cyc", 5), "×100", 1, 50, 1);
    add("mega.wireZ2Cyc", "Ciclos zona 2 (×100)", "Cable perimetral", "number", mv("wireZ2Cyc", 5), "×100", 1, 50, 1);
    add("mega.wireFwdCyc", "Ciclos avance cable", "Cable perimetral", "number", mv("wireFwdCyc", 20), "×10", 1, 200, 1);
    add("mega.wireBakCyc", "Ciclos retroceso cable", "Cable perimetral", "number", mv("wireBakCyc", 10), "×10", 1, 200, 1);
    add("mega.wireMaxTrR", "Giros der antes reinicio", "Cable perimetral", "number", mv("wireMaxTrR", 20), "×10", 1, 200, 1);
    add("mega.wireMaxTrL", "Giros izq antes reinicio", "Cable perimetral", "number", mv("wireMaxTrL", 20), "×10", 1, 200, 1);
    add("mega.cwToCharge", "CW hacia carga", "Cable perimetral", "boolean", mv("cwToCharge", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.ccwToCharge", "CCW hacia carga", "Cable perimetral", "boolean", mv("ccwToCharge", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.cwToStart", "CW hacia inicio", "Cable perimetral", "boolean", mv("cwToStart", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.ccwToStart", "CCW hacia inicio", "Cable perimetral", "boolean", mv("ccwToStart", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.compassOn", "Compás activo", "Compás", "boolean", mv("compassOn", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.compassMode", "Modo compás", "Compás", "option", mv("compassMode", 1), nullptr, 1, 3, 1,
        std::vector<std::string>{"DFRobot QMC5883", "QMC5883 Manual", "QMC5883L"});
    add("mega.compassHHold", "Mantener rumbo", "Compás", "boolean", mv("compassHHold", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.compassPower", "Potencia PID compás", "Compás", "number", mv("compassPower", 2.0), nullptr, 0.1, 10.0, 0.1);
    add("mega.compassHome", "Rumbo base (°)", "Compás", "number", mv("compassHome", 0), "°", 0, 359, 1);

    add("mega.gyroOn", "Giroscopio activo", "Giroscopio", "boolean", mv("gyroOn", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.gyroPower", "Potencia PID giroscopio", "Giroscopio", "number", mv("gyroPower", 2.0), nullptr, 0.1, 10.0, 0.1);

    add("mega.battMin", "Voltaje mínimo batería", "Batería", "number", mv("battMin", 21.0), "V", 10.0, 30.0, 0.1);
    add("mega.battSens", "Sensibilidad baja batería", "Batería", "number", mv("battSens", 5), "count", 1, 30, 1);

    add("mega.wheelAmpOn", "Sensor amperios rueda activo", "Protección ruedas", "boolean", mv("wheelAmpOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.wheelAmpMax", "Amperios máx rueda", "Protección ruedas", "number", mv("wheelAmpMax", 1.5), "A", 0.1, 10.0, 0.1);

    add("mega.bumperOn", "Bumper activo", "Bumper", "boolean", mv("bumperOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.patternMow", "Tipo de patrón", "Patrón de corte", "option", mv("patternMow", 0), nullptr, 0, 2, 1,
        std::vector<std::string>{"Aleatorio", "Paralelo", "Espiral"});

    add("mega.tiltOn", "Sensor ángulo activo", "Inclinación", "boolean", mv("tiltOn", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.tipOn", "Sensor vuelco activo", "Inclinación", "boolean", mv("tipOn", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.rainOn", "Sensor lluvia instalado", "Lluvia", "boolean", mv("rainOn", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.rainSens", "Sensibilidad lluvia", "Lluvia", "number", mv("rainSens", 5), "count", 1, 30, 1);

    add("mega.chargeStOn", "Usar base de carga", "Base de carga", "boolean", mv("chargeStOn", 1), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.alarm1On", "Alarma 1 activa", "Alarma 1", "boolean", mv("alarm1On", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.alarm1H", "Alarma 1 hora", "Alarma 1", "number", mv("alarm1H", 6), "h", 0, 23, 1);
    add("mega.alarm1M", "Alarma 1 minuto", "Alarma 1", "number", mv("alarm1M", 0), "min", 0, 59, 1);
    add("mega.alarm1Act", "Alarma 1 acción", "Alarma 1", "option", mv("alarm1Act", 4), nullptr, 1, 5, 1,
        std::vector<std::string>{"Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"});
    add("mega.alarm1Rep", "Alarma 1 repetir", "Alarma 1", "boolean", mv("alarm1Rep", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.alarm2On", "Alarma 2 activa", "Alarma 2", "boolean", mv("alarm2On", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.alarm2H", "Alarma 2 hora", "Alarma 2", "number", mv("alarm2H", 12), "h", 0, 23, 1);
    add("mega.alarm2M", "Alarma 2 minuto", "Alarma 2", "number", mv("alarm2M", 0), "min", 0, 59, 1);
    add("mega.alarm2Act", "Alarma 2 acción", "Alarma 2", "option", mv("alarm2Act", 4), nullptr, 1, 5, 1,
        std::vector<std::string>{"Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"});
    add("mega.alarm2Rep", "Alarma 2 repetir", "Alarma 2", "boolean", mv("alarm2Rep", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);

    add("mega.alarm3On", "Alarma 3 activa", "Alarma 3", "boolean", mv("alarm3On", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);
    add("mega.alarm3H", "Alarma 3 hora", "Alarma 3", "number", mv("alarm3H", 18), "h", 0, 23, 1);
    add("mega.alarm3M", "Alarma 3 minuto", "Alarma 3", "number", mv("alarm3M", 0), "min", 0, 59, 1);
    add("mega.alarm3Act", "Alarma 3 acción", "Alarma 3", "option", mv("alarm3Act", 4), nullptr, 1, 5, 1,
        std::vector<std::string>{"Zona 1", "Zona 2", "Línea", "Inicio rápido", "Personalizada"});
    add("mega.alarm3Rep", "Alarma 3 repetir", "Alarma 3", "boolean", mv("alarm3Rep", 0), nullptr, std::nullopt, std::nullopt, std::nullopt);

    return settings;
  }

  json recordingStatus() {
    const StateSnapshot snap = snapshot();
    bool can_resume = false;
    if (snap.recording && snap.recording->paused) {
      const bool fix_ok = !rec_require_rtk_fixed_ || gpsFixTypeOrNone(snap) == "fixed";
      can_resume = recordingAllowed(snap) && fix_ok;
    }
    return json{
        {"active", snap.recording.has_value()},
        {"mode", snap.recording ? json(snap.recording->mode) : json(nullptr)},
        {"points", snap.recording ? static_cast<int>(snap.recording->points.size()) : 0},
        {"lastFix", snap.recording ? json(snap.recording->last_fix) : json(nullptr)},
        {"rtkLostCount", snap.recording ? snap.recording->rtk_lost_count : 0},
        {"paused", snap.recording ? snap.recording->paused : false},
        {"pauseReason", (snap.recording && snap.recording->pause_reason) ? json(*snap.recording->pause_reason) : json(nullptr)},
        {"canResume", can_resume},
        {"bufferedMode", snap.last_recording_buffer ? json(snap.last_recording_buffer->mode) : json(nullptr)},
        {"bufferedPoints", snap.last_recording_buffer ? static_cast<int>(snap.last_recording_buffer->points.size()) : 0},
        {"pendingObstacles", static_cast<int>(snap.pending_obstacles.size())},
    };
  }

  json mapPayload() {
    const StateSnapshot snap = snapshot();
    const xbot_msgs::AbsolutePose* ui_pose = uiPoseSource(snap);
    const bool has_ui_pose = ui_pose != nullptr;
    const bool gps_llh_fresh = snap.has_gps_llh && (wallNowSec() - snap.gps_llh_seen) <= 10.0;
    json parsed = nullptr;
    if (!snap.map_json.empty()) {
      parsed = json::parse(snap.map_json, nullptr, false);
      if (parsed.is_discarded()) parsed = nullptr;
    }
    return json{
        {"map", parsed.is_null() ? json(nullptr) : parsed},
        {"raw", parsed.is_null() && !snap.map_json.empty() ? json(snap.map_json) : json(nullptr)},
        {"receivedAt", static_cast<std::int64_t>(snap.map_received_at * 1000.0)},
        {"hasMap", !parsed.is_null()},
        {"pose", json{
            {"hasPose", has_ui_pose},
            {"x", has_ui_pose ? json(static_cast<double>(ui_pose->pose.pose.position.x)) : json(nullptr)},
            {"y", has_ui_pose ? json(static_cast<double>(ui_pose->pose.pose.position.y)) : json(nullptr)},
            {"fixType", gpsFixTypeOrNone(snap)},
            {"positionAccuracy", has_ui_pose ? json(static_cast<double>(ui_pose->position_accuracy)) : json(nullptr)},
            {"lat", gps_llh_fresh ? json(snap.gps_lat) : json(nullptr)},
            {"lon", gps_llh_fresh ? json(snap.gps_lon) : json(nullptr)},
            {"altitudeM", (gps_llh_fresh && snap.has_gps_alt) ? json(snap.gps_alt) : json(nullptr)},
        }},
    };
  }

  json posePayload() {
    const StateSnapshot snap = snapshot();
    const xbot_msgs::AbsolutePose* ui_pose = uiPoseSource(snap);
    const bool has_ui_pose = ui_pose != nullptr;
    const bool gps_llh_fresh = snap.has_gps_llh && (wallNowSec() - snap.gps_llh_seen) <= 10.0;
    if (!has_ui_pose) {
      return json{
          {"hasPose", false},
          {"fixType", gpsFixTypeOrNone(snap)},
          {"lat", gps_llh_fresh ? json(snap.gps_lat) : json(nullptr)},
          {"lon", gps_llh_fresh ? json(snap.gps_lon) : json(nullptr)},
          {"altitudeM", (gps_llh_fresh && snap.has_gps_alt) ? json(snap.gps_alt) : json(nullptr)},
      };
    }

    json heading_value = nullptr;
    if (ui_pose->orientation_valid != 0) {
      double heading_deg = ui_pose->vehicle_heading * 180.0 / M_PI;
      if (heading_deg < 0.0) heading_deg += 360.0;
      heading_value = heading_deg;
    }

    return json{
        {"hasPose", has_ui_pose},
        {"x", static_cast<double>(ui_pose->pose.pose.position.x)},
        {"y", static_cast<double>(ui_pose->pose.pose.position.y)},
        {"lat", gps_llh_fresh ? json(snap.gps_lat) : json(nullptr)},
        {"lon", gps_llh_fresh ? json(snap.gps_lon) : json(nullptr)},
        {"altitudeM", (gps_llh_fresh && snap.has_gps_alt) ? json(snap.gps_alt) : json(nullptr)},
        {"headingDeg", heading_value},
        {"fixType", gpsFixTypeOrNone(snap)},
        {"positionAccuracy", static_cast<double>(ui_pose->position_accuracy)},
    };
  }

  // ----- Geometry / validation -----

  static double polygonArea(const std::vector<PointXY>& pts) {
    if (pts.size() < 3) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const PointXY& a = pts[i];
      const PointXY& b = pts[(i + 1) % pts.size()];
      sum += (a.x * b.y) - (b.x * a.y);
    }
    return sum / 2.0;
  }

  static double cross(const PointXY& o, const PointXY& p, const PointXY& q) {
    return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
  }

  static bool segmentsIntersect(const PointXY& a, const PointXY& b, const PointXY& c, const PointXY& d) {
    const double d1 = cross(c, d, a);
    const double d2 = cross(c, d, b);
    const double d3 = cross(a, b, c);
    const double d4 = cross(a, b, d);
    return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
  }

  bool isSelfIntersecting(const std::vector<PointXY>& pts) const {
    if (pts.size() < 4) return false;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const PointXY& a = pts[i];
      const PointXY& b = pts[(i + 1) % pts.size()];
      for (std::size_t j = i + 1; j < pts.size(); ++j) {
        if (std::abs(static_cast<long>(i) - static_cast<long>(j)) <= 1 ||
            (i == 0 && j == pts.size() - 1)) {
          continue;
        }
        const PointXY& c = pts[j];
        const PointXY& d = pts[(j + 1) % pts.size()];
        if (segmentsIntersect(a, b, c, d)) return true;
      }
    }
    return false;
  }

  static bool pointInPolygon(const PointXY& p, const std::vector<PointXY>& poly) {
    bool inside = false;
    std::size_t j = poly.size() - 1;
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const PointXY& a = poly[i];
      const PointXY& b = poly[j];
      if (((a.y > p.y) != (b.y > p.y)) &&
          (p.x < (b.x - a.x) * (p.y - a.y) / ((b.y - a.y) == 0.0 ? 1e-9 : (b.y - a.y)) + a.x)) {
        inside = !inside;
      }
      j = i;
    }
    return inside;
  }

  std::vector<PointXY> closePolygon(const std::vector<PointXY>& pts) const {
    if (pts.size() < 2) return pts;
    const PointXY& first = pts.front();
    const PointXY& last = pts.back();
    if (std::hypot(last.x - first.x, last.y - first.y) < rec_close_tolerance_m_) {
      return std::vector<PointXY>(pts.begin(), pts.end() - 1);
    }
    return pts;
  }

  static bool pointOnSegment(const PointXY& p, const PointXY& a, const PointXY& b, double eps = 1e-6) {
    const double c = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);
    if (std::abs(c) > eps) return false;
    const double dot = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y);
    if (dot < -eps) return false;
    const double sq_len = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    return dot - sq_len <= eps;
  }

  static bool pointOnPolygonBoundary(const PointXY& p, const std::vector<PointXY>& poly) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
      if (pointOnSegment(p, poly[i], poly[(i + 1) % poly.size()])) return true;
    }
    return false;
  }

  std::vector<PointXY> validatePolygon(const std::vector<PointXY>& points, const std::string& label) const {
    std::vector<PointXY> pts = closePolygon(points);
    if (pts.size() < 3) {
      throw std::runtime_error(label + ": necesita al menos 3 puntos.");
    }
    if (std::abs(polygonArea(pts)) < rec_min_area_m2_) {
      std::ostringstream ss;
      ss << label << ": área demasiado pequeña (< " << rec_min_area_m2_ << " m²).";
      throw std::runtime_error(ss.str());
    }
    if (isSelfIntersecting(pts)) {
      throw std::runtime_error(label + ": el polígono se cruza consigo mismo.");
    }
    if (polygonArea(pts) < 0.0) std::reverse(pts.begin(), pts.end());
    return pts;
  }

  bool polygonInsidePolygon(const std::vector<PointXY>& inner, const std::vector<PointXY>& outer) const {
    if (inner.empty() || outer.empty()) return false;
    for (const PointXY& p : inner) {
      if (!pointInPolygon(p, outer) && !pointOnPolygonBoundary(p, outer)) return false;
    }
    for (std::size_t i = 0; i < inner.size(); ++i) {
      const PointXY& a = inner[i];
      const PointXY& b = inner[(i + 1) % inner.size()];
      for (std::size_t j = 0; j < outer.size(); ++j) {
        const PointXY& c = outer[j];
        const PointXY& d = outer[(j + 1) % outer.size()];
        if (segmentsIntersect(a, b, c, d)) return false;
      }
    }
    return true;
  }

  static geometry_msgs::Polygon toRosPolygon(const std::vector<PointXY>& pts) {
    geometry_msgs::Polygon poly;
    for (const PointXY& p : pts) {
      geometry_msgs::Point32 point;
      point.x = static_cast<float>(p.x);
      point.y = static_cast<float>(p.y);
      point.z = 0.0f;
      poly.points.push_back(point);
    }
    return poly;
  }

  static geometry_msgs::Quaternion yawToQuaternion(double yaw_rad) {
    geometry_msgs::Quaternion q;
    const double half = yaw_rad / 2.0;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(half);
    q.w = std::cos(half);
    return q;
  }

  // ----- Business logic -----

  std::string gpsFixType(const xbot_msgs::AbsolutePose& pose) const {
    if (pose.source == xbot_msgs::AbsolutePose::SOURCE_GPS) {
      if ((pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0) return "fixed";
      if ((pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT) != 0) return "float";
      if ((pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK) != 0) return "float";
      return "single";
    }

    const bool has_recent =
        (pose.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE) != 0;
    if (!has_recent) return "none";
    const double accuracy = static_cast<double>(pose.position_accuracy);
    if (accuracy <= rec_fixed_accuracy_m_) return "fixed";
    if (accuracy <= rec_float_accuracy_m_) return "float";
    return "single";
  }

  static std::string gpsFixLabel(const std::string& fix_type) {
    if (fix_type == "fixed") return "RTK Fixed";
    if (fix_type == "float") return "RTK Float";
    if (fix_type == "single") return "GPS";
    return "No GPS";
  }

  const xbot_msgs::AbsolutePose* uiPoseSource(const StateSnapshot& snap) const {
    constexpr double stale_s = 10.0;
    const double now = wallNowSec();
    const bool fused_fresh = snap.has_pose && (now - snap.pose_seen) <= stale_s;
    const bool raw_fresh = snap.has_raw_gps && (now - snap.raw_gps_seen) <= stale_s;

    if (fused_fresh) return &snap.pose;
    if (raw_fresh) return &snap.raw_gps;
    if (snap.has_pose) return &snap.pose;
    if (snap.has_raw_gps) return &snap.raw_gps;
    return nullptr;
  }

  std::string gpsFixTypeOrNone(const StateSnapshot& snap) const {
    if (const xbot_msgs::AbsolutePose* pose = uiPoseSource(snap)) return gpsFixType(*pose);
    return "none";
  }

  bool recordingAllowed(const StateSnapshot& snap) const {
    if (snap.has_emergency && snap.emergency.latched_emergency) return false;
    if (snap.manual_active) return true;
    if (!snap.has_high_level) return false;
    return (snap.high_level.state & 0b11111) == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
  }

  void checkHardware(const StateSnapshot& snap, bool require_high_level = false) const {
    const double now = wallNowSec();
    const double low_age = now - snap.low_level_seen;
    if (!snap.mega_connected) {
      throw std::runtime_error(
          snap.mega_connection_status.empty() ? "Mega no conectado — sin telemetría low-level disponible."
                                             : snap.mega_connection_status);
    }
    if (!snap.has_low_level || low_age > 3.0) {
      throw std::runtime_error(
          "Mega no conectado — sin datos de /ll/mower_status. Verifica mower_mega_bridge y el cable serie.");
    }
    if (snap.has_emergency && (snap.emergency.active_emergency || snap.emergency.latched_emergency)) {
      throw std::runtime_error(
          "Emergencia activa — desactiva la parada de emergencia antes de continuar.");
    }
    const double high_age = now - snap.high_level_seen;
    if (require_high_level && (!snap.has_high_level || high_age > 5.0)) {
      throw std::runtime_error(
          "Módulo de lógica no disponible — /mower_logic/current_state sin datos. Verifica que mower_logic esté en ejecución.");
    }
  }

  void startRecording(const std::string& mode) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_.recording.has_value()) throw std::runtime_error("Ya hay una grabación activa.");
    if (mode != "mow" && mode != "nav" && mode != "exclusion") {
      throw std::runtime_error("Modo de grabación inválido: " + mode);
    }
    if (mode == "exclusion" && !state_.last_recording_buffer.has_value()) {
      throw std::runtime_error(
          "Para grabar una exclusión primero debes grabar y cerrar una zona de corte.");
    }
    const StateSnapshot snap = snapshot();
    checkHardware(snap, false);
    if (!recordingAllowed(snap)) {
      throw std::runtime_error("Sólo se puede grabar en IDLE o MANUAL y sin emergencia latched.");
    }
    RecordingSession rec;
    rec.mode = mode;
    rec.started_at = wallNowSec();
    state_.recording = rec;
    ROS_INFO("[ios_bridge] recording started: %s", mode.c_str());
  }

  RecordingSession stopRecording() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (!state_.recording.has_value()) throw std::runtime_error("No hay grabación activa.");
    RecordingSession rec = *state_.recording;
    state_.recording.reset();
    ROS_INFO("[ios_bridge] recording stopped: %s (%zu pts)", rec.mode.c_str(), rec.points.size());
    if (rec.mode == "exclusion") {
      if (rec.points.size() >= 3) state_.pending_obstacles.push_back(rec.points);
    } else {
      state_.last_recording_buffer = BufferedRecording{rec.mode, rec.points};
      if (rec.mode != "mow") state_.pending_obstacles.clear();
    }
    return rec;
  }

  void cancelRecording() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_.recording.has_value()) {
      ROS_INFO("[ios_bridge] recording cancelled");
      state_.recording.reset();
    }
  }

  void clearRecordingBuffer() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_.last_recording_buffer.reset();
    state_.pending_obstacles.clear();
  }

  void resumeRecording() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (!state_.recording.has_value()) throw std::runtime_error("No hay grabación activa.");
    RecordingSession& rec = *state_.recording;
    if (!rec.paused) return;
    const StateSnapshot snap = snapshot();
    checkHardware(snap, false);
    if (!recordingAllowed(snap)) {
      throw std::runtime_error("El robot ya no está en IDLE/MANUAL o hay una emergencia activa.");
    }
    if (rec_require_rtk_fixed_ && gpsFixTypeOrNone(snap) != "fixed") {
      throw std::runtime_error("Todavía no ha vuelto el RTK Fixed.");
    }
    rec.paused = false;
    rec.pause_reason.reset();
    rec.has_paused_at = false;
    rec.has_bad_fix_since = false;
  }

  json saveArea(const json& body, bool is_navigation) {
    const StateSnapshot snap = snapshot();
    checkHardware(snap, false);

    std::string name = body.value("name", "");
    if (name.empty()) {
      const auto now = std::time(nullptr);
      char buf[32];
      std::strftime(buf, sizeof(buf), "Area %H:%M", std::localtime(&now));
      name = buf;
    }

    const bool use_buffer = body.value("useBuffer", false);
    std::vector<PointXY> points;
    std::vector<std::vector<PointXY>> obstacles;

    if (use_buffer) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (!state_.last_recording_buffer.has_value()) {
        throw std::runtime_error("No hay grabación previa para guardar.");
      }
      const BufferedRecording& buf = *state_.last_recording_buffer;
      if ((is_navigation && buf.mode != "nav") || (!is_navigation && buf.mode != "mow")) {
        throw std::runtime_error("La grabación en buffer no coincide con el tipo solicitado.");
      }
      points = buf.points;
      if (!is_navigation) obstacles = state_.pending_obstacles;
    } else {
      if (body.contains("points")) points = parsePoints(body.at("points"));
      if (!is_navigation && body.contains("obstacles")) {
        for (const auto& obstacle_json : body.at("obstacles")) {
          obstacles.push_back(parsePoints(obstacle_json));
        }
      }
    }

    const std::vector<PointXY> validated = validatePolygon(points, "zona");
    std::vector<std::vector<PointXY>> validated_obstacles;
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
      std::vector<PointXY> ob = validatePolygon(obstacles[i], "exclusión " + std::to_string(i + 1));
      if (!polygonInsidePolygon(ob, validated)) {
        throw std::runtime_error(
            "La exclusión " + std::to_string(i + 1) + " debe quedar completamente dentro del contorno principal.");
      }
      validated_obstacles.push_back(std::move(ob));
    }

    mower_map::AddMowingAreaSrv srv;
    srv.request.area.name = name;
    srv.request.area.area = toRosPolygon(validated);
    for (const auto& obstacle : validated_obstacles) {
      srv.request.area.obstacles.push_back(toRosPolygon(obstacle));
    }
    srv.request.isNavigationArea = is_navigation;

    {
      std::lock_guard<std::recursive_mutex> lock(service_mutex_);
      awaitServiceMapUpdate("/mower_map_service/add_mowing_area",
                            [&]() {
                              if (!add_area_srv_.call(srv)) {
                                throw std::runtime_error("add_mowing_area falló.");
                              }
                            },
                            "add_mowing_area");
    }

    if (use_buffer) clearRecordingBuffer();

    return json{
        {"ok", true},
        {"name", name},
        {"points", static_cast<int>(validated.size())},
        {"obstacles", static_cast<int>(validated_obstacles.size())},
        {"isNavigationArea", is_navigation},
    };
  }

  json setDockHere() {
    const StateSnapshot snap = snapshot();
    checkHardware(snap, false);
    if (!snap.has_pose) throw std::runtime_error("Sin pose disponible.");
    if (rec_require_rtk_fixed_ && gpsFixTypeOrNone(snap) != "fixed") {
      throw std::runtime_error("Sin fix RTK Fixed — no se puede fijar dock.");
    }
    const double x = static_cast<double>(snap.pose.pose.pose.position.x);
    const double y = static_cast<double>(snap.pose.pose.pose.position.y);
    const double yaw = snap.pose.vehicle_heading;
    return callSetDock(x, y, yaw);
  }

  json setDockPose(const json& body) {
    if (!body.contains("x") || !body.contains("y")) {
      throw std::runtime_error("Faltan x/y/headingDeg.");
    }
    const double x = body.at("x").get<double>();
    const double y = body.at("y").get<double>();
    const double yaw_deg = body.value("headingDeg", 0.0);
    return callSetDock(x, y, yaw_deg * M_PI / 180.0);
  }

  json callSetDock(double x, double y, double yaw_rad) {
    mower_map::SetDockingPointSrv srv;
    srv.request.docking_pose.position.x = x;
    srv.request.docking_pose.position.y = y;
    srv.request.docking_pose.position.z = 0.0;
    srv.request.docking_pose.orientation = yawToQuaternion(yaw_rad);

    {
      std::lock_guard<std::recursive_mutex> lock(service_mutex_);
      awaitServiceMapUpdate("/mower_map_service/set_docking_point",
                            [&]() {
                              if (!set_dock_srv_.call(srv)) {
                                throw std::runtime_error("set_docking_point falló.");
                              }
                            },
                            "set_docking_point");
    }

    return json{
        {"ok", true},
        {"x", x},
        {"y", y},
        {"headingDeg", std::fmod((yaw_rad * 180.0 / M_PI) + 360.0, 360.0)},
    };
  }

  json clearFullMap() {
    {
      std::lock_guard<std::recursive_mutex> lock(service_mutex_);
      awaitServiceMapUpdate("/mower_map_service/clear_map",
                            [&]() {
                              mower_map::ClearMapSrv srv;
                              if (!clear_map_srv_.call(srv)) {
                                throw std::runtime_error("clear_map falló.");
                              }
                            },
                            "clear_map");
    }
    clearRecordingBuffer();
    return json{{"ok", true}};
  }

  json replaceMap(const json& body) {
    const StateSnapshot snap = snapshot();
    checkHardware(snap, false);

    json map_doc;
    if (body.contains("map")) {
      map_doc = body.at("map");
    } else if (body.contains("raw")) {
      const std::string raw = body.at("raw").get<std::string>();
      if (raw.empty()) throw std::runtime_error("El campo raw debe ser un JSON no vacío.");
      map_doc = json::parse(raw, nullptr, false);
      if (map_doc.is_discarded()) throw std::runtime_error("JSON inválido.");
    } else {
      throw std::runtime_error("Falta map o raw en la petición.");
    }

    if (!map_doc.is_object()) throw std::runtime_error("El mapa debe ser un objeto JSON.");
    if (!map_doc.contains("areas")) map_doc["areas"] = json::array();
    if (!map_doc.contains("docking_stations")) map_doc["docking_stations"] = json::array();
    if (!map_doc["areas"].is_array() || !map_doc["docking_stations"].is_array()) {
      throw std::runtime_error("areas y docking_stations deben ser arrays.");
    }

    {
      std::lock_guard<std::recursive_mutex> lock(service_mutex_);
      awaitMapUpdate(
          [&]() { rpcCall("map.replace", json::array({map_doc}), 3.0); },
          "map.replace");
    }

    const json updated = mapPayload();
    const json updated_map = updated.contains("map") && !updated["map"].is_null() ? updated["map"] : json::object();
    return json{
        {"ok", true},
        {"areas", updated_map.value("areas", json::array()).size()},
        {"dockingStations", updated_map.value("docking_stations", json::array()).size()},
    };
  }

  void handleCommand(const std::string& command) {
    const StateSnapshot snap = snapshot();
    ROS_INFO("[ios_bridge] command: %s", command.c_str());
    if (command == "start" || command == "exitDock") {
      setRequestedMode("auto");
      checkHardware(snap, true);
      callHighLevel(mower_msgs::HighLevelControlSrvRequest::COMMAND_START);
    } else if (command == "dock" || command == "dockMode") {
      setRequestedMode("dock");
      setManualActive(false);
      checkHardware(snap, true);
      callHighLevel(mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME);
    } else if (command == "stop" || command == "idleMode") {
      setRequestedMode("idle");
      checkHardware(snap, false);
      std_msgs::String action;
      action.data = "mower_logic:mowing/pause";
      action_pub_.publish(action);
      publishManualTwist(json{{"direction", "stop"}});
      setManualActive(false);
    } else if (command == "autoMode") {
      setRequestedMode("auto");
      setManualActive(false);
      checkHardware(snap, true);
      callHighLevel(mower_msgs::HighLevelControlSrvRequest::COMMAND_START);
    } else if (command == "manualMode") {
      setRequestedMode("manual");
      checkHardware(snap, false);
      // Pause mission before entering manual override.
      std_msgs::String action;
      action.data = "mower_logic:mowing/pause";
      action_pub_.publish(action);
      setManualActive(true, 600.0);
    } else if (command == "bladeOn") {
      checkHardware(snap, false);
      callMowerControl(true);
    } else if (command == "bladeOff") {
      checkHardware(snap, false);
      callMowerControl(false);
    } else if (command == "clearEmergency") {
      clearEmergencyStop();
      setManualActive(false);
      publishManualTwist(json{{"direction", "stop"}});
    } else {
      throw std::runtime_error("Comando '" + command + "' no reconocido.");
    }
  }

  void handleManual(const json& body) {
    checkHardware(snapshot(), false);
    setRequestedMode("manual");
    const bool active = publishManualTwist(body);
    setManualActive(active);
  }

  json handleSetting(const json& body) {
    const std::string setting_id = body.value("id", "");
    const double value = body.value("value", 0.0);

    if (setting_id == "bridge.manualLinearSpeed") {
      const double actual = clampValue(value, 0.05, 1.0);
      linear_speed_.store(actual);
      return json{{"ok", true}, {"id", setting_id}, {"value", actual}};
    }
    if (setting_id == "bridge.manualAngularSpeed") {
      const double actual = clampValue(value, 0.1, 2.0);
      angular_speed_.store(actual);
      return json{{"ok", true}, {"id", setting_id}, {"value", actual}};
    }
    if (setting_id == "bridge.udpBeacon") {
      const bool enabled = std::llround(value) != 0;
      beacon_enabled_.store(enabled);
      return json{{"ok", true}, {"id", setting_id}, {"value", enabled ? 1 : 0}};
    }
    if (setting_id.rfind("mega.", 0) == 0) {
      const std::string mega_key = setting_id.substr(5);
      std::ostringstream val;
      if (value == static_cast<int>(value)) {
        val << static_cast<int>(value);
      } else {
        val.setf(std::ios::fmtflags(0), std::ios::floatfield);
        val.precision(4);
        val << value;
      }
      std_msgs::String msg;
      msg.data = mega_key + "=" + val.str();
      {
        std::lock_guard<std::recursive_mutex> lock(service_mutex_);
        cfgset_pub_.publish(msg);
        const double confirmed = awaitMegaSettingValue(mega_key, value, 2.5);
        return json{{"ok", true}, {"id", setting_id}, {"value", confirmed}};
      }
    }
    throw std::runtime_error("Ajuste no soportado: " + setting_id);
  }

  double awaitMegaSettingValue(const std::string& mega_key, double expected, double timeout_s) {
    const double deadline = wallNowSec() + timeout_s;
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    while (wallNowSec() < deadline) {
      auto it = state_.mega_settings.find(mega_key);
      if (it != state_.mega_settings.end()) {
        try {
          const double actual = std::stod(it->second);
          if (std::abs(actual - expected) <= 0.0001) return actual;
        } catch (...) {
        }
      }
      state_cv_.wait_for(lock, std::chrono::milliseconds(50));
    }
    std::ostringstream ss;
    ss << "La Raspberry no recibió confirmación del Mega para " << mega_key
       << " dentro de " << timeout_s << " s.";
    throw std::runtime_error(ss.str());
  }

  void callHighLevel(std::uint8_t command) {
    const std::string svc = "/mower_service/high_level_control";
    if (!ros::service::waitForService(svc, ros::Duration(1.0))) {
      throw std::runtime_error("Servicio " + svc + " no disponible. ¿Está mower_logic en ejecución?");
    }
    mower_msgs::HighLevelControlSrv srv;
    srv.request.command = command;
    if (!high_level_srv_.call(srv)) {
      throw std::runtime_error("Error en " + svc);
    }
  }

  void callMowerControl(bool enabled) {
    const std::string svc = "/ll/_service/mow_enabled";
    bool done = false;
    if (ros::service::waitForService(svc, ros::Duration(1.0))) {
      mower_msgs::MowerControlSrv srv;
      srv.request.mow_enabled = enabled ? 1 : 0;
      srv.request.mow_direction = 0;
      done = mower_control_srv_.call(srv);
    }
    if (!done) {
      // Fallback path: publish direct blade command for mega_bridge.
      std_msgs::Bool msg;
      msg.data = enabled;
      blade_cmd_pub_.publish(msg);
      ROS_WARN("[ios_bridge] %s service failed, sent fallback /mega/blade_cmd", svc.c_str());
    }
  }

  void clearEmergencyStop() {
    const std::string svc = "/ll/_service/emergency";
    if (!ros::service::waitForService(svc, ros::Duration(1.0))) {
      throw std::runtime_error("Servicio " + svc + " no disponible.");
    }
    mower_msgs::EmergencyStopSrv srv;
    srv.request.emergency = 0;
    if (!emergency_stop_srv_.call(srv)) {
      throw std::runtime_error("No se pudo desactivar emergencia en " + svc);
    }
  }

  bool publishManualTwist(const json& body) {
    geometry_msgs::Twist twist;
    const json* manual = &body;
    if (body.contains("manual") && body["manual"].is_object()) {
      manual = &body["manual"];
    }

    auto readAxis = [&](const std::vector<std::string>& keys, double fallback) -> double {
      for (const auto& key : keys) {
        if (manual->contains(key) && (*manual)[key].is_number()) {
          return clampValue((*manual)[key].get<double>(), -1.0, 1.0);
        }
      }
      return fallback;
    };

    const bool has_axes = manual->contains("x") || manual->contains("y") ||
                          manual->contains("lx") || manual->contains("ly") ||
                          manual->contains("horizontal") || manual->contains("vertical") ||
                          manual->contains("joystickX") || manual->contains("joystickY");

    if (has_axes) {
      double x = readAxis({"x", "lx", "horizontal", "joystickX"}, 0.0);
      double y = readAxis({"y", "ly", "vertical", "joystickY"}, 0.0);
      const double deadband = std::max(0.0, std::min(0.30, manual_deadband_.load()));
      if (std::abs(x) < deadband) x = 0.0;
      if (std::abs(y) < deadband) y = 0.0;
      twist.linear.x = y * linear_speed_.load();
      // Invert X sign so right on joystick means right turn on robot.
      twist.angular.z = -x * angular_speed_.load();
      joy_vel_pub_.publish(twist);
      manual_cmd_vel_pub_.publish(twist);
      ROS_INFO_THROTTLE(0.5, "[ios_bridge] manual axes x=%.3f y=%.3f -> vx=%.3f wz=%.3f",
                        x, y, twist.linear.x, twist.angular.z);
      return std::abs(x) > 1e-3 || std::abs(y) > 1e-3;
    }

    std::string direction = manual->value("direction", manual->value("cmd", "stop"));
    for (char& c : direction) c = static_cast<char>(std::tolower(c));

    if (direction == "up") direction = "forward";
    else if (direction == "down" || direction == "backward") direction = "reverse";
    else if (direction == "upleft" || direction == "up_left") direction = "forward_left";
    else if (direction == "upright" || direction == "up_right") direction = "forward_right";
    else if (direction == "downleft" || direction == "down_left") direction = "reverse_left";
    else if (direction == "downright" || direction == "down_right") direction = "reverse_right";

    if (direction == "forward") {
      twist.linear.x = linear_speed_.load();
    } else if (direction == "reverse") {
      twist.linear.x = -linear_speed_.load();
    } else if (direction == "left") {
      twist.angular.z = -angular_speed_.load();
    } else if (direction == "right") {
      twist.angular.z = angular_speed_.load();
    } else if (direction == "forward_left") {
      twist.linear.x = linear_speed_.load();
      twist.angular.z = -angular_speed_.load();
    } else if (direction == "forward_right") {
      twist.linear.x = linear_speed_.load();
      twist.angular.z = angular_speed_.load();
    } else if (direction == "reverse_left") {
      twist.linear.x = -linear_speed_.load();
      twist.angular.z = -angular_speed_.load();
    } else if (direction == "reverse_right") {
      twist.linear.x = -linear_speed_.load();
      twist.angular.z = angular_speed_.load();
    } else if (direction == "stop" || direction == "center" || direction == "idle") {
      joy_vel_pub_.publish(twist);
      manual_cmd_vel_pub_.publish(twist);
      return false;
    } else {
      throw std::runtime_error("unsupported manual direction '" + direction + "'");
    }
    joy_vel_pub_.publish(twist);
    manual_cmd_vel_pub_.publish(twist);
    return true;
  }

  // ----- Wait helpers -----

  void waitForMapChange(double previous_ts, const std::string& previous_raw, const std::string& op_name) {
    const double deadline = wallNowSec() + map_update_timeout_s_;
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    while (wallNowSec() < deadline) {
      if (state_.map_received_at > previous_ts && state_.map_json != previous_raw) return;
      state_cv_.wait_for(lock, std::chrono::milliseconds(50));
    }
    std::ostringstream ss;
    ss << op_name << " ejecutado pero json_map no se actualizó dentro de "
       << map_update_timeout_s_ << " s.";
    throw std::runtime_error(ss.str());
  }

  void awaitServiceMapUpdate(const std::string& service_name,
                             const std::function<void()>& invoke,
                             const std::string& op_name) {
    const StateSnapshot snap = snapshot();
    if (!ros::service::waitForService(service_name, ros::Duration(2.0))) {
      throw std::runtime_error("Servicio " + service_name + " no disponible.");
    }
    invoke();
    waitForMapChange(snap.map_received_at, snap.map_json, op_name);
  }

  void awaitMapUpdate(const std::function<void()>& invoke, const std::string& op_name) {
    const StateSnapshot snap = snapshot();
    invoke();
    waitForMapChange(snap.map_received_at, snap.map_json, op_name);
  }

  json rpcCall(const std::string& method, const json& params, double timeout_s) {
    xbot_rpc::RpcRequest req;
    req.id = std::to_string(nowMs()) + "-" + std::to_string(++rpc_seq_);
    req.method = method;
    req.params = jsonString(params);

    auto pending = std::make_shared<RpcPending>();
    {
      std::lock_guard<std::mutex> lock(rpc_pending_mutex_);
      rpc_pending_[req.id] = pending;
    }

    rpc_request_pub_.publish(req);

    std::unique_lock<std::mutex> lock(pending->mutex);
    if (!pending->cv.wait_for(lock, std::chrono::duration<double>(timeout_s), [&]() { return pending->done; })) {
      std::lock_guard<std::mutex> pending_lock(rpc_pending_mutex_);
      rpc_pending_.erase(req.id);
      std::ostringstream ss;
      ss << "RPC " << method << " sin respuesta dentro de " << timeout_s << " s.";
      throw std::runtime_error(ss.str());
    }
    {
      std::lock_guard<std::mutex> pending_lock(rpc_pending_mutex_);
      rpc_pending_.erase(req.id);
    }
    if (pending->error) {
      std::ostringstream ss;
      ss << "RPC " << method << " falló (" << pending->code << "): " << pending->message;
      throw std::runtime_error(ss.str());
    }
    if (pending->result.empty()) return nullptr;
    json parsed = json::parse(pending->result, nullptr, false);
    if (parsed.is_discarded()) return pending->result;
    return parsed;
  }

  // ----- HTTP server -----

  void startHttpServer() {
    boost::system::error_code ec;
    tcp::endpoint endpoint(asio::ip::make_address(host_, ec), static_cast<unsigned short>(port_));
    if (ec) {
      endpoint = tcp::endpoint(tcp::v4(), static_cast<unsigned short>(port_));
    }
    acceptor_ = std::make_unique<tcp::acceptor>(io_context_);
    acceptor_->open(endpoint.protocol(), ec);
    if (ec) throw std::runtime_error("No se pudo abrir el socket HTTP: " + ec.message());
    acceptor_->set_option(asio::socket_base::reuse_address(true), ec);
    acceptor_->bind(endpoint, ec);
    if (ec) throw std::runtime_error("No se pudo bindear el socket HTTP: " + ec.message());
    acceptor_->listen(asio::socket_base::max_listen_connections, ec);
    if (ec) throw std::runtime_error("No se pudo escuchar en el socket HTTP: " + ec.message());

    http_thread_ = std::thread([this]() {
      while (running_.load() && ros::ok()) {
        tcp::socket socket(io_context_);
        boost::system::error_code accept_ec;
        acceptor_->accept(socket, accept_ec);
        if (accept_ec) {
          if (running_.load()) ROS_DEBUG("iOS bridge accept error: %s", accept_ec.message().c_str());
          continue;
        }
        std::thread(&MowerIosBridgeNode::handleHttpSession, this, std::move(socket)).detach();
      }
    });

    ROS_INFO("iOS bridge listening on http://%s:%d", host_.c_str(), port_);
  }

  void handleHttpSession(tcp::socket socket) {
    beast::flat_buffer buffer;
    boost::system::error_code ec;
    socket.set_option(tcp::no_delay(true), ec);

    for (;;) {
      http::request<http::string_body> req;
      http::read(socket, buffer, req, ec);
      if (ec == http::error::end_of_stream) break;
      if (ec) break;

      auto res = handleHttpRequest(req);
      const bool keep_alive = req.keep_alive();
      res.keep_alive(keep_alive);
      if (keep_alive) {
        res.set(http::field::connection, "keep-alive");
      } else {
        res.set(http::field::connection, "close");
      }

      http::write(socket, res, ec);
      if (ec || !keep_alive) break;
    }

    socket.shutdown(tcp::socket::shutdown_send, ec);
  }

  http::response<http::string_body> handleHttpRequest(const http::request<http::string_body>& req) {
    try {
      const std::string path = requestPath(req);
      if (!auth_token_.empty() && !authorized(req)) {
        return jsonResponse(http::status::unauthorized, json{{"error", "unauthorized"}});
      }

      if (req.method() == http::verb::options) {
        return jsonResponse(http::status::ok, json::object());
      }
      touchAppSession();

      if (req.method() == http::verb::get && path == "/api/health") {
        return jsonResponse(http::status::ok, healthPayload());
      }
      if (req.method() == http::verb::get && path == "/api/status") {
        return jsonResponse(http::status::ok, statusPayload());
      }
      if (req.method() == http::verb::get && path == "/api/telemetry") {
        return jsonResponse(http::status::ok, telemetryPayload());
      }
      if (req.method() == http::verb::get && path == "/api/settings") {
        return jsonResponse(http::status::ok, settingsPayload());
      }
      if (req.method() == http::verb::get && path == "/api/map") {
        return jsonResponse(http::status::ok, mapPayload());
      }
      if (req.method() == http::verb::get && path == "/api/pose") {
        return jsonResponse(http::status::ok, posePayload());
      }
      if (req.method() == http::verb::get && path == "/api/recording") {
        return jsonResponse(http::status::ok, recordingStatus());
      }
      if (req.method() == http::verb::get && path == "/api/map/export") {
        return jsonResponse(http::status::ok, mapPayload());
      }

      if (req.method() == http::verb::post && path == "/api/command") {
        const json body = readJsonBody(req.body());
        try {
          handleCommand(body.value("command", ""));
          return jsonResponse(http::status::ok, json{{"ok", true}});
        } catch (const std::exception& exc) {
          ROS_WARN("[ios_bridge] command '%s' rejected: %s", body.value("command", "").c_str(), exc.what());
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/manual") {
        const json body = readJsonBody(req.body());
        try {
          handleManual(body);
          return jsonResponse(http::status::ok, json{{"ok", true}});
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/emergency/clear") {
        try {
          handleCommand("clearEmergency");
          return jsonResponse(http::status::ok, json{{"ok", true}});
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/settings") {
        const json body = readJsonBody(req.body());
        try {
          return jsonResponse(http::status::ok, handleSetting(body));
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/recording/start") {
        const json body = readJsonBody(req.body());
        try {
          startRecording(body.value("mode", ""));
          return jsonResponse(http::status::ok, json{{"ok", true}, {"recording", recordingStatus()}});
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/recording/stop") {
        try {
          RecordingSession rec = stopRecording();
          return jsonResponse(
              http::status::ok,
              json{{"ok", true},
                   {"mode", rec.mode},
                   {"points", static_cast<int>(rec.points.size())},
                   {"recording", recordingStatus()}});
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/recording/resume") {
        try {
          resumeRecording();
          return jsonResponse(http::status::ok, json{{"ok", true}, {"recording", recordingStatus()}});
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/recording/cancel") {
        cancelRecording();
        clearRecordingBuffer();
        return jsonResponse(http::status::ok, json{{"ok", true}});
      }
      if (req.method() == http::verb::post && path == "/api/areas/mowing") {
        const json body = readJsonBody(req.body());
        try {
          return jsonResponse(http::status::ok, saveArea(body, false));
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/areas/navigation") {
        const json body = readJsonBody(req.body());
        try {
          return jsonResponse(http::status::ok, saveArea(body, true));
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/dock/here") {
        try {
          return jsonResponse(http::status::ok, setDockHere());
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/dock") {
        const json body = readJsonBody(req.body());
        try {
          return jsonResponse(http::status::ok, setDockPose(body));
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post && path == "/api/map/clear") {
        const json body = readJsonBody(req.body());
        if (!body.value("confirm", false)) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", "Falta confirmación."}});
        }
        try {
          return jsonResponse(http::status::ok, clearFullMap());
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }
      if (req.method() == http::verb::post &&
          (path == "/api/map/replace" || path == "/api/map/import")) {
        const json body = readJsonBody(req.body());
        try {
          return jsonResponse(http::status::ok, replaceMap(body));
        } catch (const std::exception& exc) {
          return jsonResponse(http::status::ok, json{{"ok", false}, {"error", exc.what()}});
        }
      }

      return jsonResponse(http::status::not_found, json{{"error", "not_found"}});
    } catch (const std::exception& exc) {
      ROS_WARN("iOS bridge request failed: %s", exc.what());
      return jsonResponse(http::status::internal_server_error, json{{"error", exc.what()}});
    }
  }

  static json readJsonBody(const std::string& body) {
    if (body.empty()) return json::object();
    json parsed = json::parse(body, nullptr, false);
    if (parsed.is_discarded()) throw std::runtime_error("JSON inválido.");
    return parsed;
  }

  std::string requestPath(const http::request<http::string_body>& req) const {
    std::string path = std::string(req.target());
    const auto q = path.find('?');
    if (q != std::string::npos) path.resize(q);
    return path;
  }

  bool authorized(const http::request<http::string_body>& req) const {
    auto it = req.find(http::field::authorization);
    return it != req.end() && it->value() == ("Bearer " + auth_token_);
  }

  http::response<http::string_body> jsonResponse(http::status status, const json& payload) const {
    http::response<http::string_body> res{status, 11};
    res.set(http::field::content_type, "application/json");
    res.set("Access-Control-Allow-Origin", "*");
    res.set("Access-Control-Allow-Headers", "Authorization, Content-Type");
    res.set("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    res.body() = jsonString(payload);
    res.prepare_payload();
    return res;
  }

  // ----- Utility helpers -----

  std::string localIp() const {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return "127.0.0.1";
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(80);
    ::inet_pton(AF_INET, "8.8.8.8", &dst.sin_addr);
    std::string result = "127.0.0.1";
    if (::connect(sock, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) == 0) {
      sockaddr_in name{};
      socklen_t len = sizeof(name);
      if (::getsockname(sock, reinterpret_cast<sockaddr*>(&name), &len) == 0) {
        char buffer[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &name.sin_addr, buffer, sizeof(buffer))) result = buffer;
      }
    }
    ::close(sock);
    return result;
  }

  void udpBeaconLoop() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;
    int enable = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    ::setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(beacon_port_));
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
      ROS_WARN("[ios_bridge] UDP beacon bind failed on port %d. Continuing with ephemeral source port.",
               beacon_port_);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(beacon_port_));
    addr.sin_addr.s_addr = INADDR_BROADCAST;

    while (running_.load() && ros::ok()) {
      if (beacon_enabled_.load()) {
        const std::string auth = auth_token_.empty() ? "open" : "token";
        const std::string payload =
            "MOWER|" + discovery_name_ + "|" + localIp() + ":" + std::to_string(port_) + "|auth=" + auth;
        const auto len = static_cast<socklen_t>(sizeof(addr));
        if (::sendto(sock, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&addr), len) < 0) {
          ROS_DEBUG("iOS bridge UDP beacon failed");
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    ::close(sock);
  }

  static double firstValid(std::initializer_list<double> values) {
    for (double value : values) {
      if (value > 0.0) return value;
    }
    return 0.0;
  }

  static int rpmToPwm(int rpm) {
    if (rpm == 0) return 0;
    return static_cast<int>(clampValue(static_cast<double>(std::abs(rpm)) / 20.0, 1.0, 255.0));
  }

  static std::vector<PointXY> parsePoints(const json& points_json) {
    std::vector<PointXY> points;
    if (!points_json.is_array()) throw std::runtime_error("Lista de puntos inválida.");
    for (const auto& item : points_json) {
      if (!item.is_array() || item.size() < 2) throw std::runtime_error("Punto inválido.");
      points.push_back(PointXY{item.at(0).get<double>(), item.at(1).get<double>()});
    }
    return points;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_ios_bridge");
  MowerIosBridgeNode bridge;
  bridge.start();
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  bridge.stop();
  return 0;
}
