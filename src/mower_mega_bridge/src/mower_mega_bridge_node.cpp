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
#include <cmath>
#include <cstdint>
#include <iomanip>
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
        , volts_(0.0)
        , amps_(0.0)
        , wheel_amps_(0.0)
        , charging_(0)
        , pwm_l_(0)
        , pwm_r_(0)
        , mega_state_("IDLE")
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        port_       = pnh.param<std::string>("port", "/dev/ttyAMA2");
        baud_       = pnh.param<int>("baud", 57600);
        wheel_dist_ = pnh.param<double>("wheel_distance_m", 0.325);
        max_pwm_    = pnh.param<int>("max_pwm", 255);
        hb_hz_      = pnh.param<double>("heartbeat_hz", 3.0);

        pub_emergency_ = nh.advertise<mower_msgs::Emergency>  ("ll/emergency",    1);
        pub_status_    = nh.advertise<mower_msgs::Status>     ("ll/mower_status", 1);
        pub_power_     = nh.advertise<mower_msgs::Power>      ("ll/power",        1);
        pub_twist_     = nh.advertise<geometry_msgs::TwistStamped>(
                             "ll/diff_drive/measured_twist", 1);
        pub_esc_l_     = nh.advertise<mower_msgs::ESCStatus>(
                             "ll/diff_drive/left_esc_status", 1);
        pub_esc_r_     = nh.advertise<mower_msgs::ESCStatus>(
                             "ll/diff_drive/right_esc_status", 1);

        sub_cmd_vel_ = nh.subscribe("ll/cmd_vel", 1,
                           &MegaBridge::cbCmdVel, this,
                           ros::TransportHints().tcpNoDelay());
        sub_hl_      = nh.subscribe("/mower_logic/current_state", 1,
                           &MegaBridge::cbHighLevel, this);

        srv_mow_ = nh.advertiseService("ll/_service/mow_enabled",
                       &MegaBridge::srvMowEnabled, this);
        srv_em_  = nh.advertiseService("ll/_service/emergency",
                       &MegaBridge::srvEmergency, this);

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

    // ── Atomic flags (no mutex needed) ────────────────────────────────────────
    std::atomic<uint32_t> seq_;
    std::atomic<bool>     emergency_;
    std::atomic<bool>     mow_enabled_;
    std::atomic<bool>     local_avoid_;
    std::atomic<bool>     ser_open_;

    // ── Sensor state (guarded by state_mutex_) ────────────────────────────────
    std::mutex  state_mutex_;
    double      volts_, amps_, wheel_amps_;
    int         charging_;
    int         pwm_l_, pwm_r_;
    std::string mega_state_;

    // ── Serial (write path guarded by write_mutex_) ───────────────────────────
    serial::Serial ser_;
    std::mutex     write_mutex_;

    // ── ROS handles ───────────────────────────────────────────────────────────
    ros::Publisher     pub_emergency_, pub_status_, pub_power_;
    ros::Publisher     pub_twist_, pub_esc_l_, pub_esc_r_;
    ros::Subscriber    sub_cmd_vel_, sub_hl_;
    ros::ServiceServer srv_mow_, srv_em_;

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
                ROS_INFO("[mega_bridge] serial open: %s", port_.c_str());
                return;
            } catch (serial::IOException& e) {
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
                try { ser_.close(); } catch (...) {}
            } catch (serial::IOException& e) {
                ROS_WARN("[mega_bridge] IO error: %s", e.what());
                ser_open_ = false;
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

        if (type == "HB") {
            // alive

        } else if (type == "BATT") {
            mower_msgs::Power pm;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                volts_ = fields.empty() ? 0.0 : std::stod(fields[0]);
                pm.battery_voltage_chg = static_cast<float>(volts_);
                pm.charge_current      = static_cast<float>(charging_);
            }
            pub_power_.publish(pm);

        } else if (type == "AMPS") {
            std::lock_guard<std::mutex> lk(state_mutex_);
            amps_ = fields.empty() ? 0.0 : std::stod(fields[0]);

        } else if (type == "WHEEL_AMPS") {
            mower_msgs::ESCStatus esc;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                wheel_amps_ = fields.empty() ? 0.0 : std::stod(fields[0]);
                esc.status  = (mega_state_ == "RUNNING")
                              ? mower_msgs::ESCStatus::ESC_STATUS_RUNNING
                              : mower_msgs::ESCStatus::ESC_STATUS_OK;
                esc.current = static_cast<float>(wheel_amps_ / 2.0);
            }
            pub_esc_l_.publish(esc);
            pub_esc_r_.publish(esc);

        } else if (type == "CHARGE") {
            mower_msgs::Power pm;
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                charging_ = fields.empty() ? 0 : std::stoi(fields[0]);
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
                    pwm_l_ = std::stoi(fields[0]);
                    pwm_r_ = std::stoi(fields[1]);
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
            }

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

    // ── ROS services ──────────────────────────────────────────────────────────

    bool srvMowEnabled(mower_msgs::MowerControlSrv::Request&  req,
                       mower_msgs::MowerControlSrv::Response&)
    {
        mow_enabled_ = req.mow_enabled;
        send("CMD", {"BLADE", req.mow_enabled ? "ON" : "OFF"});
        return true;
    }

    bool srvEmergency(mower_msgs::EmergencyStopSrv::Request&  req,
                      mower_msgs::EmergencyStopSrv::Response&)
    {
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
