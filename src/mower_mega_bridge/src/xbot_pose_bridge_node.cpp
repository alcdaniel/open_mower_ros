#include <cmath>
#include <mutex>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <xbot_msgs/AbsolutePose.h>
#include <xbot_positioning/GPSControlSrv.h>
#include <xbot_positioning/SetPoseSrv.h>

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q_msg) {
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

class XbotPoseBridgeNode {
 public:
  XbotPoseBridgeNode() : nh_(), pnh_("~") {
    raw_gps_timeout_s_ = pnh_.param<double>("raw_gps_timeout_s", 2.0);
    heading_offset_deg_ = pnh_.param<double>("heading_offset_deg", 180.0);
    pose_pub_ = nh_.advertise<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", 10, true);
    filtered_map_pub_ = nh_.advertise<nav_msgs::Odometry>("/odometry/filtered_map", 10, true);

    odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &XbotPoseBridgeNode::cbOdom, this);
    raw_gps_sub_ = nh_.subscribe("/ll/position/gps", 10, &XbotPoseBridgeNode::cbRawGps, this);

    gps_srv_ = nh_.advertiseService("xbot_positioning/set_gps_state",
                                    &XbotPoseBridgeNode::srvSetGpsState, this);
    set_pose_srv_ = nh_.advertiseService("xbot_positioning/set_robot_pose",
                                         &XbotPoseBridgeNode::srvSetRobotPose, this);

    ROS_INFO("[xbot_pose_bridge] publishing /xbot_positioning/xb_pose and /odometry/filtered_map from /odometry/filtered + /ll/position/gps");
  }

 private:
  void cbRawGps(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_raw_gps_ = *msg;
    has_raw_gps_ = true;
    last_raw_gps_wall_ = ros::Time::now();
  }

  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    xbot_msgs::AbsolutePose pose;
    bool used_raw_gps_position = false;
    xbot_msgs::AbsolutePose raw_gps_copy;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (gps_enabled_ && has_raw_gps_ &&
          (ros::Time::now() - last_raw_gps_wall_).toSec() <= raw_gps_timeout_s_) {
        pose = last_raw_gps_;
        raw_gps_copy = last_raw_gps_;
        used_raw_gps_position = true;
      }
    }

    // Keep absolute position from raw GPS when available, but always derive
    // heading and motion vector from the filtered odometry/IMU chain.
    const double raw_yaw = yawFromQuaternion(msg->pose.pose.orientation);
    const double corrected_yaw =
        normalizeAngle(raw_yaw + heading_offset_deg_ * M_PI / 180.0);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z));
    tf2::Quaternion odom_q;
    tf2::fromMsg(msg->pose.pose.orientation, odom_q);
    odom_to_base.setRotation(odom_q);

    tf2::Transform map_to_odom;
    bool has_map_to_odom = false;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (used_raw_gps_position) {
        tf2::Transform map_to_base;
        map_to_base.setOrigin(tf2::Vector3(raw_gps_copy.pose.pose.position.x,
                                           raw_gps_copy.pose.pose.position.y,
                                           raw_gps_copy.pose.pose.position.z));
        tf2::Quaternion map_q;
        tf2::fromMsg(quaternionFromYaw(corrected_yaw), map_q);
        map_to_base.setRotation(map_q);
        last_map_to_odom_ = map_to_base * odom_to_base.inverse();
        has_map_to_odom_ = true;
      }

      has_map_to_odom = has_map_to_odom_;
      if (has_map_to_odom_) {
        map_to_odom = last_map_to_odom_;
      }
    }

    tf2::Transform map_to_base =
        has_map_to_odom ? (map_to_odom * odom_to_base) : odom_to_base;

    pose.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    pose.header.frame_id = "map";
    pose.sensor_stamp = 0;
    pose.received_stamp = 0;
    pose.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    pose.flags = 0;
    pose.orientation_valid = 1;
    pose.motion_vector_valid = 1;
    pose.pose.pose.position.x = map_to_base.getOrigin().x();
    pose.pose.pose.position.y = map_to_base.getOrigin().y();
    pose.pose.pose.position.z = map_to_base.getOrigin().z();
    pose.pose.pose.orientation = quaternionFromYaw(corrected_yaw);
    pose.vehicle_heading = corrected_yaw;
    pose.motion_heading = corrected_yaw;
    pose.motion_vector.x = msg->twist.twist.linear.x * std::cos(corrected_yaw);
    pose.motion_vector.y = msg->twist.twist.linear.x * std::sin(corrected_yaw);
    pose.motion_vector.z = 0.0;
    pose.orientation_accuracy = 0.2f;

    if (used_raw_gps_position) {
      pose.pose.covariance = raw_gps_copy.pose.covariance;
      pose.position_accuracy = raw_gps_copy.position_accuracy;
      pose.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else if (has_map_to_odom) {
      pose.pose.covariance = msg->pose.covariance;
      pose.position_accuracy = 999.0f;
      pose.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
    } else {
      pose.pose = msg->pose;
      pose.position_accuracy = 999.0f;
      pose.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_pose_ = pose.pose.pose;
      has_pose_ = true;
    }

    publishFilteredMap(msg, pose, corrected_yaw, has_map_to_odom, used_raw_gps_position, raw_gps_copy);
    pose_pub_.publish(pose);
  }

  void publishFilteredMap(const nav_msgs::Odometry::ConstPtr& local_odom,
                          const xbot_msgs::AbsolutePose& global_pose,
                          double corrected_yaw,
                          bool has_map_to_odom,
                          bool used_raw_gps_position,
                          const xbot_msgs::AbsolutePose& raw_gps_copy) {
    nav_msgs::Odometry filtered_map = *local_odom;
    filtered_map.header.stamp = ros::Time::now();
    filtered_map.header.frame_id = "map";
    filtered_map.child_frame_id = "base_link";
    filtered_map.pose.pose.orientation = quaternionFromYaw(corrected_yaw);

    filtered_map.pose.pose.position.x = global_pose.pose.pose.position.x;
    filtered_map.pose.pose.position.y = global_pose.pose.pose.position.y;
    filtered_map.pose.pose.position.z = global_pose.pose.pose.position.z;
    if (used_raw_gps_position) {
      filtered_map.pose.covariance = raw_gps_copy.pose.covariance;
    }

    if (has_map_to_odom) {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = filtered_map.header.stamp;
      tf_msg.header.frame_id = "map";
      tf_msg.child_frame_id = "odom";
      {
        std::lock_guard<std::mutex> lk(mutex_);
        tf_msg.transform = tf2::toMsg(last_map_to_odom_);
      }
      tf_broadcaster_.sendTransform(tf_msg);
    }

    filtered_map_pub_.publish(filtered_map);
  }

  bool srvSetGpsState(xbot_positioning::GPSControlSrvRequest& req,
                      xbot_positioning::GPSControlSrvResponse& res) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      gps_enabled_ = req.gps_enabled;
    }
    ROS_INFO("[xbot_pose_bridge] gps_enabled set to %s", gps_enabled_ ? "true" : "false");
    return true;
  }

  bool srvSetRobotPose(xbot_positioning::SetPoseSrvRequest& req,
                       xbot_positioning::SetPoseSrvResponse& res) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_pose_ = req.robot_pose;
      has_pose_ = true;
    }
    ROS_WARN("[xbot_pose_bridge] set_robot_pose acknowledged but not applied to EKF state");
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pose_pub_;
  ros::Publisher filtered_map_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber raw_gps_sub_;
  ros::ServiceServer gps_srv_;
  ros::ServiceServer set_pose_srv_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::mutex mutex_;
  bool gps_enabled_ = true;
  bool has_raw_gps_ = false;
  bool has_pose_ = false;
  bool has_map_to_odom_ = false;
  double raw_gps_timeout_s_ = 2.0;
  double heading_offset_deg_ = 180.0;
  ros::Time last_raw_gps_wall_;
  xbot_msgs::AbsolutePose last_raw_gps_;
  tf2::Transform last_map_to_odom_;
  geometry_msgs::Pose last_pose_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xbot_pose_bridge");
  XbotPoseBridgeNode node;
  ros::spin();
  return 0;
}
