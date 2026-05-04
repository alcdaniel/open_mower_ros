/**
 * calibrate_odom_gain_node.cpp
 *
 * C++ ROS node to calibrate ODOM_VX_GAIN constant in Mega's Movement_Control.ino.
 *
 * Procedure:
 *   1. Place mower on a clear straight path (5+ m).
 *   2. Optionally measure with tape: mark start position.
 *   3. Run this node. It will:
 *        - Read initial /odom position
 *        - Command vx for N seconds via /ll/manual_cmd_vel (mode=1, manual)
 *        - Read final /odom and /fix (GPS) if enabled
 *        - Compute gain = real_distance / odom_reported_distance
 *        - Print recommended ODOM_VX_GAIN value
 *   4. Pass _tape:=<meters> for manually measured distance (most accurate).
 *   5. Update ODOM_VX_GAIN in Movement_Control.ino and reflash.
 *
 * Parameters (private ~):
 *   vx          (double, 0.5)   Linear velocity command (m/s)
 *   duration    (double, 8.0)   Seconds to run
 *   gps_topic   (string, /fix)  GPS topic name
 *   use_gps     (bool, false)   Use GPS for reference distance
 *   tape        (double, -1.0)  Manually measured distance (m); >0 enables tape mode
 *   current_gain (double, 0.85) Current ODOM_VX_GAIN value flashed on Mega
 *
 * Usage:
 *   rosrun mower_mega_bridge calibrate_odom_gain_node \
 *       _vx:=0.5 _duration:=8.0 _tape:=4.20 _current_gain:=0.85
 *
 *   rosrun mower_mega_bridge calibrate_odom_gain_node \
 *       _use_gps:=true _gps_topic:=/fix _vx:=0.5 _duration:=15.0
 *
 * Topics:
 *   Subscribed:
 *     /odom              nav_msgs/Odometry      (Mega dead-reckoning)
 *     /fix or custom     sensor_msgs/NavSatFix  (optional GPS)
 *   Published:
 *     /ll/manual_cmd_vel geometry_msgs/Twist    (mode=1 manual on Mega)
 */

#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

namespace
{
constexpr double kEarthRadius = 6371000.0;  // m
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double d) { return d * kPi / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2)
{
    double lat1r = deg2rad(lat1);
    double lat2r = deg2rad(lat2);
    double dlat  = deg2rad(lat2 - lat1);
    double dlon  = deg2rad(lon2 - lon1);
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1r) * std::cos(lat2r) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2.0 * kEarthRadius * std::asin(std::sqrt(a));
}
}  // namespace

class OdomCalibrator
{
public:
    OdomCalibrator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), have_odom_(false), have_gps_(false)
    {
        // Parameters
        vx_           = pnh.param<double>("vx",           0.5);
        duration_     = pnh.param<double>("duration",     8.0);
        gps_topic_    = pnh.param<std::string>("gps_topic", "/fix");
        use_gps_      = pnh.param<bool>("use_gps",        false);
        tape_         = pnh.param<double>("tape",         -1.0);
        current_gain_ = pnh.param<double>("current_gain", 0.85);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/ll/manual_cmd_vel", 1);
        sub_odom_ = nh_.subscribe("/odom", 10, &OdomCalibrator::cbOdom, this);

        if (use_gps_ && tape_ < 0.0) {
            sub_gps_ = nh_.subscribe(gps_topic_, 10, &OdomCalibrator::cbGps, this);
            ROS_INFO("[calib] subscribing GPS: %s", gps_topic_.c_str());
        }

        ROS_INFO("[calib] params: vx=%.2f m/s  duration=%.1fs  tape=%.2f m  use_gps=%s  current_gain=%.3f",
                 vx_, duration_, tape_, use_gps_ ? "true" : "false", current_gain_);
    }

    void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_odom_x_ = msg->pose.pose.position.x;
        last_odom_y_ = msg->pose.pose.position.y;
        last_odom_t_ = ros::Time::now();
        have_odom_ = true;
    }

    void cbGps(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;
        if (msg->status.status < 0) return;  // No fix
        std::lock_guard<std::mutex> lk(mtx_);
        last_gps_lat_ = msg->latitude;
        last_gps_lon_ = msg->longitude;
        have_gps_ = true;
    }

    bool waitForOdom(double timeout_s = 5.0)
    {
        ros::Time t0 = ros::Time::now();
        ros::Rate r(20);
        while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout_s) {
            ros::spinOnce();
            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (have_odom_) return true;
            }
            r.sleep();
        }
        return false;
    }

    bool waitForGps(double timeout_s = 10.0)
    {
        if (!use_gps_) return true;
        ros::Time t0 = ros::Time::now();
        ros::Rate r(10);
        while (ros::ok() && (ros::Time::now() - t0).toSec() < timeout_s) {
            ros::spinOnce();
            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (have_gps_) return true;
            }
            r.sleep();
        }
        return false;
    }

    void publishStop()
    {
        geometry_msgs::Twist zero;
        for (int i = 0; i < 5 && ros::ok(); ++i) {
            cmd_pub_.publish(zero);
            ros::Duration(0.05).sleep();
        }
    }

    void run()
    {
        ROS_INFO("[calib] waiting for /odom (max 5s) ...");
        if (!waitForOdom(5.0)) {
            ROS_ERROR("[calib] no /odom received — is mower_mega_bridge_node running?");
            return;
        }

        if (use_gps_ && tape_ < 0.0) {
            ROS_INFO("[calib] waiting for GPS fix (max 10s) ...");
            if (!waitForGps(10.0)) {
                ROS_WARN("[calib] no GPS fix; results without --tape will be invalid");
            }
        }

        // Snapshot start state
        double start_x, start_y, start_lat = 0.0, start_lon = 0.0;
        bool have_start_gps;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            start_x = last_odom_x_;
            start_y = last_odom_y_;
            have_start_gps = have_gps_;
            if (have_gps_) {
                start_lat = last_gps_lat_;
                start_lon = last_gps_lon_;
            }
        }

        ROS_INFO("[calib] === CALIBRATION START ===");
        ROS_INFO("[calib] vx=%.2f m/s  duration=%.1fs  expected_distance=%.2f m",
                 vx_, duration_, vx_ * duration_);
        ROS_INFO("[calib] start odom=(%.3f, %.3f)", start_x, start_y);
        if (have_start_gps) {
            ROS_INFO("[calib] start gps=(%.7f, %.7f)", start_lat, start_lon);
        }

        ROS_INFO("[calib] 3 second countdown — clear path NOW!");
        for (int i = 3; i > 0 && ros::ok(); --i) {
            ROS_INFO("[calib]   %d ...", i);
            ros::Duration(1.0).sleep();
        }

        // Command vx loop at 15 Hz
        geometry_msgs::Twist twist;
        twist.linear.x  = vx_;
        twist.angular.z = 0.0;

        ros::Rate rate(15.0);
        ros::Time t0 = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - t0).toSec() < duration_) {
            cmd_pub_.publish(twist);
            ros::spinOnce();
            rate.sleep();
        }

        publishStop();
        ros::Duration(0.5).sleep();  // let final odom arrive
        ros::spinOnce();

        // Snapshot end state
        double end_x, end_y, end_lat = 0.0, end_lon = 0.0;
        bool have_end_gps;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            end_x = last_odom_x_;
            end_y = last_odom_y_;
            have_end_gps = have_gps_;
            if (have_gps_) {
                end_lat = last_gps_lat_;
                end_lon = last_gps_lon_;
            }
        }

        // Compute distances
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double odom_dist = std::hypot(dx, dy);

        ROS_INFO("[calib] === RESULTS ===");
        ROS_INFO("[calib] odom reported distance: %.3f m  (dx=%.3f, dy=%.3f)",
                 odom_dist, dx, dy);

        double real_dist = -1.0;
        std::string source;

        if (tape_ > 0.0) {
            real_dist = tape_;
            source = "tape";
        } else if (use_gps_ && have_start_gps && have_end_gps) {
            real_dist = haversine(start_lat, start_lon, end_lat, end_lon);
            source = "GPS haversine";
        } else {
            ROS_WARN("[calib] no real-distance reference (no _tape, no GPS).");
            ROS_WARN("[calib] Re-run with _tape:=<meters> or _use_gps:=true");
            return;
        }

        ROS_INFO("[calib] real distance (%s): %.3f m", source.c_str(), real_dist);

        if (odom_dist < 0.01) {
            ROS_ERROR("[calib] odom distance too small (%.3f m). Mower didn't move? Check Wheels_Activate.", odom_dist);
            return;
        }

        double ratio = real_dist / odom_dist;
        double new_gain = current_gain_ * ratio;

        // Pretty output to stdout (so it's not muffled by rosout filters)
        std::ostringstream out;
        out << "\n";
        out << "================ CALIBRATION REPORT ================\n";
        out << "  current ODOM_VX_GAIN .... " << std::fixed << std::setprecision(3) << current_gain_ << "\n";
        out << "  odom reported .......... " << std::fixed << std::setprecision(3) << odom_dist << " m\n";
        out << "  real distance (" << source << ") " << std::fixed << std::setprecision(3) << real_dist << " m\n";
        out << "  ratio (real/reported) .. " << std::fixed << std::setprecision(4) << ratio << "\n";
        out << "  ----------------------------------------------\n";
        out << "  RECOMMENDED ODOM_VX_GAIN  " << std::fixed << std::setprecision(3) << new_gain << "\n";
        out << "====================================================\n";
        out << "\n";
        out << "Apply: edit arduino/MEGA_V9.751/Movement_Control.ino:\n";
        out << "    static float ODOM_VX_GAIN = " << std::fixed << std::setprecision(3) << new_gain << ";\n";
        out << "Then reflash Mega and re-run calibration.\n";
        out << "Target: ratio close to 1.000\n\n";

        std::cout << out.str() << std::flush;
        ROS_INFO_STREAM(out.str());
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher  cmd_pub_;
    ros::Subscriber sub_odom_, sub_gps_;

    std::mutex mtx_;
    bool have_odom_;
    bool have_gps_;
    double last_odom_x_ = 0.0, last_odom_y_ = 0.0;
    ros::Time last_odom_t_;
    double last_gps_lat_ = 0.0, last_gps_lon_ = 0.0;

    // Params
    double      vx_;
    double      duration_;
    std::string gps_topic_;
    bool        use_gps_;
    double      tape_;
    double      current_gain_;
};

// Graceful shutdown — stop motors on Ctrl-C
namespace {
OdomCalibrator* g_calibrator = nullptr;

void sigintHandler(int /*sig*/)
{
    if (g_calibrator) g_calibrator->publishStop();
    ros::shutdown();
}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_odom_gain", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OdomCalibrator cal(nh, pnh);
    g_calibrator = &cal;
    std::signal(SIGINT, sigintHandler);

    cal.run();
    cal.publishStop();
    return 0;
}
