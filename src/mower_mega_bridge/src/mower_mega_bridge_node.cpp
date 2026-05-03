/**
 * mower_mega_bridge_node.cpp
 *
 * Serial bridge between Raspberry Pi (ROS) and Arduino Mega.
 * Publishes the same interface as mower_comms_v1.
 *
 * Topics subscribed:
 *   ll/cmd_vel                      geometry_msgs/Twist
 *   /mower_logic/current_state      mower_msgs/HighLevelStatus
 *
 * Topics published:
 *   ll/emergency                    mower_msgs/Emergency
 *   ll/mower_status                 mower_msgs/Status
 *   ll/power                        mower_msgs/Power
 *   ll/diff_drive/measured_twist    geometry_msgs/TwistStamped
 *   ll/diff_drive/left_esc_status   mower_msgs/ESCStatus
 *   ll/diff_drive/right_esc_status  mower_msgs/ESCStatus
 *   mega/sonar/front                sensor_msgs/Range   (cm → metres, 999 → ∞)
 *   mega/sonar/left                 sensor_msgs/Range
 *   mega/sonar/right                sensor_msgs/Range
 *   mega/bumper                     std_msgs/Bool
 *   mega/rain                       std_msgs/Bool
 *   mega/tilt                       std_msgs/Bool
 *   mega/wire_detected              std_msgs/Bool
 *   mega/imu                        sensor_msgs/Imu     (yaw only)
 *   mega/cfg                        std_msgs/String     (key=value, one per setting)
 *   mega/cfg_loaded                 std_msgs/Bool       (true when full dump received)
 *
 * Topics subscribed (in addition to above):
 *   mega/cfgget                     std_msgs/Bool       (publish true to request settings)
 *   mega/cfgset                     std_msgs/String     (publish "key=value" to write one)
 *
 * Services:
 *   ll/_service/mow_enabled         mower_msgs/MowerControlSrv
 *   ll/_service/emergency           mower_msgs/EmergencyStopSrv
 *
 * Parameters (private ~):
 *   port             /dev/ttyAMA2
 *   baud             57600
 *   wheel_distance_m 0.325
 *   max_pwm          255
 *   heartbeat_hz     3.0
 */

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/Power.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// ── Protocol helpers ──────────────────────────────────────────────────────────

static uint8_t xorCs(const std::string& body)
{
    uint8_t cs = 0;
    for (unsigned char c : body) cs ^= c;
    return cs;
}

static bool parseMsg(const std::string& raw,
                     std::string& type,
                     std::vector<std::string>& fields)
{
    std::string line = raw;
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
        line.pop_back();

    if (line.size() < 4 || line[0] != '$') return false;

    auto star = line.rfind('*');
    if (star == std::string::npos) return false;

    std::string body   = line.substr(1, star - 1);
    std::string cs_str = line.substr(star + 1);

    uint8_t got;
    try { got = static_cast<uint8_t>(std::stoul(cs_str, nullptr, 16)); }
    catch (...) { return false; }

    if (xorCs(body) != got) {
        ROS_WARN_THROTTLE(5, "[mega_bridge] bad checksum: %s", line.c_str());
        return false;
    }

    std::istringstream ss(body);
    std::string tok;
    std::vector<std::string> parts;
    while (std::getline(ss, tok, ','))
        parts.push_back(tok);

    if (parts.empty()) return false;

    type = parts[0];
    fields.clear();
    // drop last token (sequence number)
    for (std::size_t i = 1; i + 1 < parts.size(); ++i)
        fields.push_back(parts[i]);
    return true;
}

static bool parseDoubleField(const std::vector<std::string>& fields,
                             std::size_t idx,
                             double fallback,
                             double& out)
{
    if (idx >= fields.size()) {
        out = fallback;
        return true;
    }
    const std::string& s = fields[idx];
    if (s.empty() || s == "?" || s == "nan" || s == "NaN" || s == "NAN") {
        out = fallback;
        return false;
    }
    try {
        out = std::stod(s);
        return true;
    } catch (...) {
        out = fallback;
        return false;
    }
}

static bool parseIntField(const std::vector<std::string>& fields,
                          std::size_t idx,
                          int fallback,
                          int& out)
{
    if (idx >= fields.size()) {
        out = fallback;
        return true;
    }
    const std::string& s = fields[idx];
    if (s.empty() || s == "?" || s == "nan" || s == "NaN" || s == "NAN") {
        out = fallback;
        return false;
    }
    try {
        out = std::stoi(s);
        return true;
    } catch (...) {
        out = fallback;
        return false;
    }
}

// ── Bridge class ──────────────────────────────────────────────────────────────

class MegaBridge
{
public:
    MegaBridge()
        : seq_(0)
        , emergency_(false)
        , mow_enabled_(false)
        , local_avoid_(false)
        , ser_open_(false)
        , mega_connected_(false)
        , volts_(0.0)
        , amps_(0.0)
        , wheel_amps_(0.0)
        , charging_(0)
        , pwm_l_(0)
        , pwm_r_(0)
        , mega_state_("IDLE")
        , compass_deg_(0.0)
        , bumper_(false)
        , rain_(false)
        , tilt_(false)
        , wire_(false)
        , sonar_{999, 999, 999}
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        port_       = pnh.param<std::string>("port", "/dev/ttyAMA0");
        baud_       = pnh.param<int>("baud", 57600);
        wheel_dist_ = pnh.param<double>("wheel_distance_m", 0.325);
        max_pwm_    = pnh.param<int>("max_pwm", 255);
        hb_hz_      = pnh.param<double>("heartbeat_hz", 3.0);
        offline_publish_hz_ = pnh.param<double>("offline_publish_hz", 1.0);
        rx_timeout_s_       = pnh.param<double>("rx_timeout_s", 3.0);

        // Latched so late subscribers (e.g. mower_logic during startup) always
        // receive an initial emergency state and do not block waiting forever.
        pub_emergency_ = nh.advertise<mower_msgs::Emergency>  ("ll/emergency",    1, true);
        pub_status_    = nh.advertise<mower_msgs::Status>     ("ll/mower_status", 1);
        pub_power_     = nh.advertise<mower_msgs::Power>      ("ll/power",        1);
        pub_twist_     = nh.advertise<geometry_msgs::TwistStamped>(
                             "ll/diff_drive/measured_twist", 1);
        pub_esc_l_     = nh.advertise<mower_msgs::ESCStatus>(
                             "ll/diff_drive/left_esc_status", 1);
        pub_esc_r_     = nh.advertise<mower_msgs::ESCStatus>(
                             "ll/diff_drive/right_esc_status", 1);

        pub_sonar_[0]  = nh.advertise<sensor_msgs::Range>("mega/sonar/front", 1);
        pub_sonar_[1]  = nh.advertise<sensor_msgs::Range>("mega/sonar/left",  1);
        pub_sonar_[2]  = nh.advertise<sensor_msgs::Range>("mega/sonar/right", 1);
        pub_bumper_    = nh.advertise<std_msgs::Bool>("mega/bumper",       1);
        pub_rain_      = nh.advertise<std_msgs::Bool>("mega/rain",         1);
        pub_tilt_      = nh.advertise<std_msgs::Bool>("mega/tilt",         1);
        pub_wire_      = nh.advertise<std_msgs::Bool>("mega/wire_detected",1);
        pub_compass_imu_ = nh.advertise<sensor_msgs::Imu>("mega/imu",     1);
        pub_cfg_        = nh.advertise<std_msgs::String>("mega/cfg",        10);
        pub_cfg_loaded_ = nh.advertise<std_msgs::Bool>  ("mega/cfg_loaded",  1);
        pub_connected_  = nh.advertise<std_msgs::Bool>("mega/connected", 1, true);
        pub_connection_status_ =
            nh.advertise<std_msgs::String>("mega/connection_status", 1, true);

        sub_cfgget_ = nh.subscribe("mega/cfgget", 1, &MegaBridge::cbCfgGet, this);
        sub_cfgset_ = nh.subscribe("mega/cfgset", 1, &MegaBridge::cbCfgSet, this);

        sub_cmd_vel_ = nh.subscribe("ll/cmd_vel", 1,
                           &MegaBridge::cbCmdVel, this,
                           ros::TransportHints().tcpNoDelay());
        sub_hl_      = nh.subscribe("/mower_logic/current_state", 1,
                           &MegaBridge::cbHighLevel, this);

        srv_mow_ = nh.advertiseService("ll/_service/mow_enabled",
                       &MegaBridge::srvMowEnabled, this);
        srv_em_  = nh.advertiseService("ll/_service/emergency",
                       &MegaBridge::srvEmergency, this);

        updateConnectionState(false, "MEGA_DISCONNECTED: esperando telemetria del Mega en " + port_);
        publishDisconnectedState();
        status_timer_ = nh.createTimer(
            ros::Duration(1.0 / std::max(0.2, offline_publish_hz_)),
            &MegaBridge::cbStatusTimer,
            this);
        reader_thread_ = std::thread(&MegaBridge::serialReader, this);
        hb_thread_     = std::thread(&MegaBridge::heartbeatLoop, this);

        ROS_INFO("[mega_bridge] started  port=%s  baud=%d", port_.c_str(), baud_);
    }

    ~MegaBridge()
    {
        if (reader_thread_.joinable()) reader_thread_.detach();
        if (hb_thread_.joinable())     hb_thread_.detach();
    }

    void spin() { ros::spin(); }

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    std::string port_;
    int         baud_;
    double      wheel_dist_;
    int         max_pwm_;
    double      hb_hz_;
    double      offline_publish_hz_;
    double      rx_timeout_s_;

    // ── Atomic flags (no mutex needed) ────────────────────────────────────────
    std::atomic<uint32_t> seq_;
    std::atomic<bool>     emergency_;
    std::atomic<bool>     mow_enabled_;
    std::atomic<bool>     local_avoid_;
    std::atomic<bool>     ser_open_;
    std::atomic<bool>     mega_connected_;

    // ── Sensor state (guarded by state_mutex_) ────────────────────────────────
    std::mutex  state_mutex_;
    double      volts_, amps_, wheel_amps_;
    int         charging_;
    int         pwm_l_, pwm_r_;
    std::string mega_state_;
    double      compass_deg_;
    bool        bumper_, rain_, tilt_, wire_;
    int         sonar_[3];

    // ── Serial (write path guarded by write_mutex_) ───────────────────────────
    serial::Serial ser_;
    std::mutex     write_mutex_;

    std::mutex  connection_mutex_;
    std::string connection_status_;
    ros::Time   last_rx_time_;

    // ── ROS handles ───────────────────────────────────────────────────────────
    ros::Publisher     pub_emergency_, pub_status_, pub_power_;
    ros::Publisher     pub_twist_, pub_esc_l_, pub_esc_r_;
    ros::Publisher     pub_sonar_[3], pub_bumper_, pub_rain_, pub_tilt_, pub_wire_;
    ros::Publisher     pub_compass_imu_;
    ros::Publisher     pub_cfg_, pub_cfg_loaded_;
    ros::Publisher     pub_connected_, pub_connection_status_;
    ros::Subscriber    sub_cmd_vel_, sub_hl_, sub_cfgget_, sub_cfgset_;
    ros::ServiceServer srv_mow_, srv_em_;
    ros::Timer         status_timer_;

    std::thread reader_thread_, hb_thread_;

    // ── Protocol ──────────────────────────────────────────────────────────────

    std::string buildMsg(const std::string& type,
                         const std::vector<std::string>& fields)
    {
        uint32_t seq = ++seq_;
        std::ostringstream body;
        body << type;
        for (const auto& f : fields) body << ',' << f;
        body << ',' << seq;
        const std::string b = body.str();

        std::ostringstream msg;
        msg << '$' << b << '*'
            << std::uppercase << std::hex
            << std::setw(2) << std::setfill('0')
            << static_cast<int>(xorCs(b)) << '\n';
        return msg.str();
    }

    void send(const std::string& type,
              const std::vector<std::string>& fields = {})
    {
        if (!ser_open_) return;
        std::string msg = buildMsg(type, fields);
        std::lock_guard<std::mutex> lk(write_mutex_);
        if (!ser_open_) return;
        try {
            ser_.write(msg);
        } catch (serial::SerialException& e) {
            ROS_WARN("[mega_bridge] write error: %s", e.what());
            ser_open_ = false;
            updateConnectionState(false, "MEGA_DISCONNECTED: error de escritura serie (" + std::string(e.what()) + ")");
            try { ser_.close(); } catch (...) {}
        }
    }

    void updateConnectionState(bool connected, const std::string& status)
    {
        {
            std::lock_guard<std::mutex> lk(connection_mutex_);
            connection_status_ = status;
            if (connected) last_rx_time_ = ros::Time::now();
        }
        mega_connected_ = connected;

        std_msgs::Bool bm;
        bm.data = connected;
        pub_connected_.publish(bm);

        std_msgs::String sm;
        sm.data = status;
        pub_connection_status_.publish(sm);
    }

    std::string connectionStatus()
    {
        std::lock_guard<std::mutex> lk(connection_mutex_);
        return connection_status_;
    }

    void markRx()
    {
        {
            std::lock_guard<std::mutex> lk(connection_mutex_);
            last_rx_time_ = ros::Time::now();
        }
        if (!mega_connected_.load()) {
            updateConnectionState(true, "MEGA_CONNECTED: telemetria recibida desde " + port_);
        }
    }

    void publishDisconnectedState()
    {
        const auto now = ros::Time::now();
        const auto reason = connectionStatus();

        mower_msgs::Status status;
        status.stamp = now;
        status.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
        status.raspberry_pi_power = true;
        status.is_charging = false;
        status.esc_power = false;
        status.rain_detected = false;
        status.sound_module_available = false;
        status.sound_module_busy = false;
        status.ui_board_available = false;
        status.mow_enabled = false;
        status.mower_esc_status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
        status.mower_esc_temperature = 0.0f;
        status.mower_esc_current = 0.0f;
        status.mower_motor_temperature = 0.0f;
        status.mower_motor_rpm = 0.0f;
        pub_status_.publish(status);

        mower_msgs::Power power;
        power.stamp = now;
        power.charge_voltage_adc = 0.0f;
        power.charge_voltage_chg = 0.0f;
        power.charge_current = 0.0f;
        power.battery_voltage_adc = 0.0f;
        power.battery_voltage_chg = 0.0f;
        power.battery_voltage_bms = 0.0f;
        power.battery_current = 0.0f;
        power.battery_pct = 0.0f;
        power.battery_soc = 0.0f;
        power.battery_temp = 0.0f;
        power.dcdc_input_current = 0.0f;
        power.charger_input_current = 0.0f;
        power.charger_status = "MEGA_DISCONNECTED";
        power.charger_enabled = false;
        power.bms_status = "MEGA_DISCONNECTED";
        power.bms_extra_data = reason;
        pub_power_.publish(power);

        mower_msgs::Emergency emergency;
        emergency.stamp = now;
        emergency.active_emergency = false;
        emergency.latched_emergency = false;
        emergency.reason = reason;
        pub_emergency_.publish(emergency);

        mower_msgs::ESCStatus esc;
        esc.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
        esc.current = 0.0f;
        esc.tacho = 0;
        esc.rpm = 0;
        esc.temperature_motor = 0.0f;
        esc.temperature_pcb = 0.0f;
        pub_esc_l_.publish(esc);
        pub_esc_r_.publish(esc);
    }

    void cbStatusTimer(const ros::TimerEvent&)
    {
        if (ser_open_.load()) {
            ros::Time last_rx;
            {
                std::lock_guard<std::mutex> lk(connection_mutex_);
                last_rx = last_rx_time_;
            }
            if (!last_rx.isZero() &&
                (ros::Time::now() - last_rx) > ros::Duration(rx_timeout_s_)) {
                ser_open_ = false;
                try { ser_.close(); } catch (...) {}
                updateConnectionState(
                    false,
                    "MEGA_DISCONNECTED: sin telemetria del Mega durante " +
                    std::to_string(static_cast<int>(rx_timeout_s_)) + " s");
            }
        }

        if (!mega_connected_.load()) {
            publishDisconnectedState();
        } else {
            std_msgs::Bool bm;
            bm.data = true;
            pub_connected_.publish(bm);
        }
    }

    // ── Serial open ───────────────────────────────────────────────────────────

    void openSerial()
    {
        while (ros::ok()) {
            try {
                {
                    std::lock_guard<std::mutex> lk(write_mutex_);
                    if (ser_.isOpen()) ser_.close();
                    ser_.setPort(port_);
                    ser_.setBaudrate(static_cast<uint32_t>(baud_));
                    // Short timeout so read loop releases write_mutex quickly
                    serial::Timeout to = serial::Timeout::simpleTimeout(100);
                    ser_.setTimeout(to);
                    ser_.open();
                    ser_open_ = true;
                }
                updateConnectionState(false, "MEGA_WAITING: puerto serie abierto en " + port_ + ", esperando datos");
                ROS_INFO("[mega_bridge] serial open: %s", port_.c_str());
                return;
            } catch (serial::IOException& e) {
                updateConnectionState(false, "MEGA_DISCONNECTED: no se puede abrir " + port_ + " (" + e.what() + ")");
                ROS_WARN_THROTTLE(10, "[mega_bridge] cannot open %s: %s",
                                  port_.c_str(), e.what());
                ros::Duration(2.0).sleep();
            }
        }
    }

    // ── Serial reader thread ──────────────────────────────────────────────────

    void serialReader()
    {
        openSerial();
        while (ros::ok()) {
            if (!ser_open_) { openSerial(); continue; }
            try {
                // readline does NOT hold write_mutex_ — only the read path uses ser_
                // serial::Serial read/write are internally serialised via its own mutex
                std::string line = ser_.readline(256, "\n");
                if (!line.empty()) handleLine(line);
            } catch (serial::SerialException& e) {
                ROS_WARN("[mega_bridge] read error: %s", e.what());
                ser_open_ = false;
                updateConnectionState(false, "MEGA_DISCONNECTED: error de lectura serie (" + std::string(e.what()) + ")");
                try { ser_.close(); } catch (...) {}
            } catch (serial::IOException& e) {
                ROS_WARN("[mega_bridge] IO error: %s", e.what());
                ser_open_ = false;
                updateConnectionState(false, "MEGA_DISCONNECTED: error IO serie (" + std::string(e.what()) + ")");
                try { ser_.close(); } catch (...) {}
            }
        }
    }

    // ── Message dispatcher ────────────────────────────────────────────────────

    void handleLine(const std::string& line)
    {
        std::string type;
        std::vector<std::string> fields;
        if (!parseMsg(line, type, fields)) return;
        markRx();

        if (type == "HB") {
            // alive

        } else if (type == "BATT") {
            mower_msgs::Power pm;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                if (!parseDoubleField(fields, 0, volts_, volts_)) {
                    ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid BATT field: '%s'",
                                      fields.empty() ? "" : fields[0].c_str());
                }
                pm.battery_voltage_chg = static_cast<float>(volts_);
                pm.charge_current      = static_cast<float>(charging_);
            }
            pub_power_.publish(pm);

        } else if (type == "AMPS") {
            std::lock_guard<std::mutex> lk(state_mutex_);
            if (!parseDoubleField(fields, 0, amps_, amps_)) {
                ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid AMPS field: '%s'",
                                  fields.empty() ? "" : fields[0].c_str());
            }

        } else if (type == "WHEEL_AMPS") {
            mower_msgs::ESCStatus escL, escR;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                if (!parseDoubleField(fields, 0, wheel_amps_, wheel_amps_)) {
                    ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid WHEEL_AMPS field: '%s'",
                                      fields.empty() ? "" : fields[0].c_str());
                }
                auto s = (mega_state_ == "RUNNING")
                         ? mower_msgs::ESCStatus::ESC_STATUS_RUNNING
                         : mower_msgs::ESCStatus::ESC_STATUS_OK;
                escL.status  = escR.status  = s;
                escL.current = escR.current = static_cast<float>(wheel_amps_ / 2.0);
                // Derive RPM from last known PWM so the iOS bridge can compute motor %
                escL.rpm = pwm_l_ * 20;
                escR.rpm = pwm_r_ * 20;
            }
            pub_esc_l_.publish(escL);
            pub_esc_r_.publish(escR);

        } else if (type == "CHARGE") {
            mower_msgs::Power pm;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                if (!parseIntField(fields, 0, charging_, charging_)) {
                    ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid CHARGE field: '%s'",
                                      fields.empty() ? "" : fields[0].c_str());
                }
                pm.battery_voltage_chg = static_cast<float>(volts_);
                pm.charge_current      = static_cast<float>(charging_);
            }
            pub_power_.publish(pm);

        } else if (type == "PWM") {
            if (fields.size() >= 2) {
                geometry_msgs::TwistStamped ts;
                ts.header.stamp = ros::Time::now();
                {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    int pl = pwm_l_;
                    int pr = pwm_r_;
                    if (!parseIntField(fields, 0, pwm_l_, pl) ||
                        !parseIntField(fields, 1, pwm_r_, pr)) {
                        ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid PWM fields: '%s','%s'",
                                          fields[0].c_str(), fields[1].c_str());
                    }
                    pwm_l_ = pl;
                    pwm_r_ = pr;
                    double vl = static_cast<double>(pwm_l_) / max_pwm_;
                    double vr = static_cast<double>(pwm_r_) / max_pwm_;
                    ts.twist.linear.x  = (vl + vr) / 2.0;
                    ts.twist.angular.z = (vr - vl) / wheel_dist_;
                }
                pub_twist_.publish(ts);
            }

        } else if (type == "STATE") {
            std::string state = fields.empty() ? "" : fields[0];
            mower_msgs::Status sm;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                mega_state_ = state;
                sm.mower_status =
                    (state == "IDLE" || state == "PARKED" || state == "DOCKED")
                    ? mower_msgs::Status::MOWER_STATUS_INITIALIZING
                    : mower_msgs::Status::MOWER_STATUS_OK;
                sm.mow_enabled = mow_enabled_.load();
                sm.is_charging = (charging_ > 0);
            }
            if      (state == "LOCAL_AVOIDANCE") local_avoid_ = true;
            else if (state == "READY" || state == "IDLE" ||
                     state == "PARKED" || state == "DOCKED")
                local_avoid_ = false;
            pub_status_.publish(sm);

        } else if (type == "EVT") {
            std::string kind = fields.empty() ? "" : fields[0];
            if (kind == "OBSTACLE") {
                local_avoid_ = true;
                mower_msgs::Emergency em;
                em.active_emergency  = false;
                em.latched_emergency = false;
                std::string reason = "OBSTACLE";
                for (std::size_t i = 1; i < fields.size(); ++i)
                    reason += ':' + fields[i];
                em.reason = reason;
                pub_emergency_.publish(em);
                ROS_INFO_THROTTLE(1, "[mega_bridge] obstacle: %s", reason.c_str());
            } else if (kind == "SAFETY") {
                std::string reason = "SAFETY";
                for (std::size_t i = 1; i < fields.size(); ++i)
                    reason += ':' + fields[i];
                setRosEmergency(true, reason);
            } else if (kind == "DEADMAN") {
                ROS_WARN("[mega_bridge] Mega triggered deadman stop");
                setRosEmergency(true, "DEADMAN");
            }

        } else if (type == "SONAR") {
            if (fields.size() >= 3) {
                auto now = ros::Time::now();
                const char* frames[3] = {"sonar_front", "sonar_left", "sonar_right"};
                {
                    std::lock_guard<std::mutex> lk(state_mutex_);
                    for (int i = 0; i < 3; ++i) {
                        int sv = sonar_[i];
                        if (!parseIntField(fields, i, sonar_[i], sv)) {
                            ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid SONAR[%d] field: '%s'",
                                              i, fields[i].c_str());
                        }
                        sonar_[i] = sv;
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    sensor_msgs::Range r;
                    r.header.stamp    = now;
                    r.header.frame_id = frames[i];
                    r.radiation_type  = sensor_msgs::Range::ULTRASOUND;
                    r.field_of_view   = 0.26f;   // ~15°
                    r.min_range       = 0.02f;
                    r.max_range       = 3.00f;
                    r.range = (sonar_[i] >= 999)
                              ? std::numeric_limits<float>::infinity()
                              : sonar_[i] / 100.0f;
                    pub_sonar_[i].publish(r);
                }
            }

        } else if (type == "BUMPER") {
            bool b = !fields.empty() && fields[0] == "1";
            { std::lock_guard<std::mutex> lk(state_mutex_); bumper_ = b; }
            std_msgs::Bool bm; bm.data = b;
            pub_bumper_.publish(bm);

        } else if (type == "RAIN") {
            bool r = !fields.empty() && fields[0] != "0";
            { std::lock_guard<std::mutex> lk(state_mutex_); rain_ = r; }
            std_msgs::Bool rm; rm.data = r;
            pub_rain_.publish(rm);

        } else if (type == "TILT") {
            bool t = !fields.empty() && fields[0] != "0";
            { std::lock_guard<std::mutex> lk(state_mutex_); tilt_ = t; }
            std_msgs::Bool tm; tm.data = t;
            pub_tilt_.publish(tm);
            if (t) {
                ROS_WARN("[mega_bridge] Tilt sensor triggered — triggering emergency");
                setRosEmergency(true, "TILT_SENSOR");
            }

        } else if (type == "WIRE") {
            bool w = !fields.empty() && fields[0] != "0";
            { std::lock_guard<std::mutex> lk(state_mutex_); wire_ = w; }
            std_msgs::Bool wm; wm.data = w;
            pub_wire_.publish(wm);

        } else if (type == "COMPASS") {
            double deg = compass_deg_;
            if (!parseDoubleField(fields, 0, compass_deg_, deg)) {
                ROS_WARN_THROTTLE(5.0, "[mega_bridge] invalid COMPASS field: '%s'",
                                  fields.empty() ? "" : fields[0].c_str());
            }
            { std::lock_guard<std::mutex> lk(state_mutex_); compass_deg_ = deg; }
            // Publish as IMU (yaw only) — robot_localization / EKF can fuse this.
            sensor_msgs::Imu imu;
            imu.header.stamp    = ros::Time::now();
            imu.header.frame_id = "base_link";
            constexpr double kPi = 3.14159265358979323846;
            double rad = deg * kPi / 180.0;
            imu.orientation.x = 0.0;
            imu.orientation.y = 0.0;
            imu.orientation.z = std::sin(rad / 2.0);
            imu.orientation.w = std::cos(rad / 2.0);
            // Roll/pitch unknown; yaw variance ~0.05 rad² (≈±12°)
            imu.orientation_covariance[0] = 1e6;
            imu.orientation_covariance[4] = 1e6;
            imu.orientation_covariance[8] = 0.05;
            imu.angular_velocity_covariance[0]    = -1;  // not available
            imu.linear_acceleration_covariance[0] = -1;  // not available
            pub_compass_imu_.publish(imu);

        } else if (type == "CFG") {
            // fields[0] = key, fields[1] = value
            if (fields.size() >= 2) {
                std_msgs::String sm;
                sm.data = fields[0] + "=" + fields[1];
                pub_cfg_.publish(sm);
            }

        } else if (type == "CFGEND") {
            std_msgs::Bool bm;
            bm.data = true;
            pub_cfg_loaded_.publish(bm);
            ROS_INFO("[mega_bridge] settings dump complete");

        } else if (type == "ERR") {
            std::string msg;
            for (const auto& f : fields) msg += f + ' ';
            ROS_WARN("[mega_bridge] Mega error: %s", msg.c_str());
        }
    }

    // ── Emergency ─────────────────────────────────────────────────────────────

    void setRosEmergency(bool active, const std::string& reason = "")
    {
        emergency_ = active;
        send("CMD", {"ESTOP", active ? "1" : "0"});
        mower_msgs::Emergency em;
        em.active_emergency  = active;
        em.latched_emergency = active;
        em.reason = active ? reason : "";
        pub_emergency_.publish(em);
    }

    // ── ROS callbacks ─────────────────────────────────────────────────────────

    void cbCmdVel(const geometry_msgs::Twist::ConstPtr& twist)
    {
        if (emergency_ || local_avoid_) return;

        double vx = twist->linear.x;
        double wz = twist->angular.z;
        double sr = std::max(-1.0, std::min(1.0, vx + 0.5 * wheel_dist_ * wz));
        double sl = std::max(-1.0, std::min(1.0, vx - 0.5 * wheel_dist_ * wz));

        send("CMD", {"NAV",
                     std::to_string(static_cast<int>(sl * max_pwm_)),
                     std::to_string(static_cast<int>(sr * max_pwm_))});
    }

    void cbHighLevel(const mower_msgs::HighLevelStatus::ConstPtr&) {}

    void cbCfgGet(const std_msgs::Bool::ConstPtr&)
    {
        send("CMD", {"CFGGET"});
    }

    void cbCfgSet(const std_msgs::String::ConstPtr& msg)
    {
        // Expect "key=value"
        auto eq = msg->data.find('=');
        if (eq == std::string::npos) {
            ROS_WARN("[mega_bridge] cfgset bad format: %s", msg->data.c_str());
            return;
        }
        std::string key = msg->data.substr(0, eq);
        std::string val = msg->data.substr(eq + 1);
        send("CMD", {"CFGSET", key, val});
    }

    // ── ROS services ──────────────────────────────────────────────────────────

    bool srvMowEnabled(mower_msgs::MowerControlSrv::Request&  req,
                       mower_msgs::MowerControlSrv::Response&)
    {
        if (!mega_connected_.load()) {
            ROS_WARN_THROTTLE(2, "[mega_bridge] rejecting mow_enabled: Mega not connected");
            return false;
        }
        mow_enabled_ = req.mow_enabled;
        send("CMD", {"BLADE", req.mow_enabled ? "ON" : "OFF"});
        return true;
    }

    bool srvEmergency(mower_msgs::EmergencyStopSrv::Request&  req,
                      mower_msgs::EmergencyStopSrv::Response&)
    {
        if (!mega_connected_.load() && !static_cast<bool>(req.emergency)) {
            ROS_WARN_THROTTLE(2, "[mega_bridge] rejecting emergency reset: Mega not connected");
            return false;
        }
        setRosEmergency(static_cast<bool>(req.emergency), "ROS_REQUEST");
        return true;
    }

    // ── Heartbeat loop ────────────────────────────────────────────────────────

    void heartbeatLoop()
    {
        ros::Rate rate(hb_hz_);
        while (ros::ok()) {
            send("HB", {"RPI"});
            rate.sleep();
        }
    }
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mower_mega_bridge");
    MegaBridge bridge;
    bridge.spin();
    return 0;
}
