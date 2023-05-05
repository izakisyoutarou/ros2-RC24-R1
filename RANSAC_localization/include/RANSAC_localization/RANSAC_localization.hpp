
#pragma once

#include <iostream>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

#include "utilities/can_utils.hpp"

#include "RANSAC_localization/config.hpp"
#include "RANSAC_localization/pose_fuser.hpp"
#include "RANSAC_localization/detect_lines.hpp"
#include "RANSAC_localization/detect_circles.hpp"
#include "RANSAC_localization/converter.hpp"
#include "RANSAC_localization/voxel_grid_filter.hpp"

#include "visibility.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

namespace self_localization{
class RANSACLocalization : public rclcpp::Node{
public:
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PoseFuser pose_fuser;  // センサ融合器
  DtectLines detect_lines;
  DetectCircles detect_circles;
  Converter converter;
  VoxelGridFilter voxel_grid_filter;
  void callback_restart(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
  void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void init();
  void create_ER_map();
  void create_RR_map();
  double semi_circle(const double &x, const double &r){return sqrt(r * r - x * x);}
  void generate_circle(vector<LaserPoint> &points, const Vector3d &circle, int num_points);
  void create_map_line(vector<LaserPoint> &points, const double &start_map_point, const double &end_map_point, const double &static_map_point, const char coordinate);
  void publishers(vector<LaserPoint> &points);
  Vector3d calc_body_to_sensor(const Vector6d& sensor_pos);
  void update(const Vector3d &estimated, const Vector3d &laser_estimated, const Vector3d &current_scan_odom, const Vector3d &scan_odom_motion, vector<LaserPoint> &points);
  void correction(const Vector3d &scan_odom_motion, const Vector3d &estimated, const Vector3d &current_scan_odom, const Vector3d &diff_circle);
  void get_correction_rate_average(const Vector3d &estimated, const Vector3d &laser_estimated, const Vector3d &current_scan_odom);
  double calc_correction_rate_average(const double &estimated_, const double &laser_estimated_, const double &current_scan_odom_, double &correction_rate_sum_, int &correction_count_);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr restart_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransaced_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr self_pose_publisher;
  rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr init_angle_publisher;

  geometry_msgs::msg::Vector3 vector_msg;
  sensor_msgs::msg::PointCloud2 ER_map_cloud;
  sensor_msgs::msg::PointCloud2 RR_map_cloud;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;
  geometry_msgs::msg::PoseStamped odom_stamped;
  rclcpp::QoS _qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::QoS fast_qos = rclcpp::QoS(rclcpp::KeepLast(1));


  vector<LaserPoint> ER_map_points;
  vector<LaserPoint> RR_map_points;
  Vector3d init_pose = Vector3d::Zero();
  Vector3d odom = Vector3d::Zero();
  Vector3d last_odom = Vector3d::Zero();
  Vector3d diff_odom = Vector3d::Zero();
  Vector3d est_diff_sum = Vector3d::Zero();
  Vector3d last_estimated = Vector3d::Zero();
  Vector6d tf_laser2robot = Vector6d::Zero();


  double last_odom_received_time=0.0;
  double last_jy_received_time=0.0;
  double last_scan_received_time=0.0;
  double dt_scan=0.0;

  Vector3d correction_rate_ave = Vector3d::Zero();
  Vector3d correction_rate_sum = Vector3d::Zero();
  Vector3i correction_count = Vector3i::Zero();

  chrono::system_clock::time_point time_start, time_end, amendment_permission_time_start;

  bool plot_mode_;
  string robot_type_;
  int count=0;

  vector<double> initial_pose_;
  vector<double> second_initial_pose_;
};
}
