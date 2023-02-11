#include "icp_base_slam/icp_base_slam.hpp"

namespace self_localization{
IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "START");
  plot_mode_ = this->get_parameter("plot_mode").as_bool();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  laser_weight_ = this->get_parameter("laser_weight").as_double();
  odom_weight_ = this->get_parameter("odom_weight").as_double();
  trial_num_ = this->get_parameter("trial_num").as_int();
  inlier_dist_threshold_ = this->get_parameter("inlier_dist_threshold").as_double();

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::scan_callback, this, placeholders::_1));

  odom_linear_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_100",
    _qos,
    std::bind(&IcpBaseSlam::callback_odom_linear, this, std::placeholders::_1)
  );
  odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_101",
    _qos,
    std::bind(&IcpBaseSlam::callback_odom_angular, this, std::placeholders::_1)
  );

  pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/filtered_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  map_pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/map_point_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  ransac_pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/ransac_point_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  path_publisher = this->create_publisher<nav_msgs::msg::Path>(
    "self_localization/path",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());


  ransac_lines = RansacLines(trial_num_, inlier_dist_threshold_);
  cloud.points.resize(view_ranges/reso);
  input_elephant_cloud.points.resize(51655);
  inlier_cloud.points.resize(2161);

  voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

  create_elephant_map();
  init.x = init_pose_x;
  init.y = init_pose_y;
  odom = init;
  last_scan_odom = init;
  estimated_odom = init;
  last_estimated = init;
}

void IcpBaseSlam::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double x = (double)bytes_to_float(_candata);
  double y = (double)bytes_to_float(_candata+4);
  diff_odom.x = x - last_odom.x;
  diff_odom.y = y - last_odom.y;
  last_odom.x = x;
  last_odom.y = y;
  odom.x += diff_odom.x;
  odom.y += diff_odom.y;
  estimated_odom.x = odom.x + diff_estimated_sum.x;
  estimated_odom.y = odom.y + diff_estimated_sum.y;
}

void IcpBaseSlam::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double yaw = (double)bytes_to_float(_candata);
  diff_odom.yaw = yaw - last_odom.yaw;
  odom.yaw += diff_odom.yaw;
  last_odom.yaw = odom.yaw;
  estimated_odom.yaw = normalize_yaw(odom.yaw + diff_estimated_sum.yaw);
}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  scan_execution_time_start = chrono::system_clock::now();
  double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_scan = current_scan_received_time - last_scan_received_time;
  last_scan_received_time = current_scan_received_time;
  if (dt_scan > 0.03 /* [sec] */) {
    // RCLCPP_WARN(this->get_logger(), "scan time interval is too large->%f", dt_scan);
  }
  current_scan_odom = estimated_odom;
  double odom_to_lidar_x = odom_to_lidar_length * cos(current_scan_odom.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(current_scan_odom.yaw);
  vector<config::LaserPoint> src_points;
  for(size_t i=0; i< msg->ranges.size(); ++i) {
    config::LaserPoint src_point;
    if(msg->ranges[i] > 14 || msg->ranges[i] < 0.5) continue;
    src_point.angle = msg->angle_min + msg->angle_increment * i - current_scan_odom.yaw;
    src_point.dist = msg->ranges[i];
    src_point.x = src_point.dist * cos(src_point.angle) + current_scan_odom.x + odom_to_lidar_x;
    src_point.y = -src_point.dist * sin(src_point.angle) + current_scan_odom.y + odom_to_lidar_y;
    src_points.push_back(src_point);
  }

  ransac_lines.fuse_inliers(src_points, current_scan_odom, odom_to_lidar_x, odom_to_lidar_y);
  vector<config::LaserPoint> line_points = ransac_lines.get_sum();
  diff_estimated = ransac_lines.get_estimated_diff();
  diff_estimated_sum += diff_estimated;

  inlier_cloud.points.resize(line_points.size());
  for(size_t i=0; i<line_points.size(); i++){
    inlier_cloud.points[i].x = line_points[i].x;
    inlier_cloud.points[i].y = line_points[i].y;
  }

  if(plot_mode_) pointcloud2_view(input_elephant_cloud, inlier_cloud);

  scan_execution_time_end = chrono::system_clock::now();
  scan_execution_time = chrono::duration_cast<chrono::milliseconds>(scan_execution_time_end-scan_execution_time_start).count();
  // RCLCPP_INFO(this->get_logger(), "scan execution time->%d", scan_execution_time);
}

double IcpBaseSlam::quaternionToYaw(double x, double y, double z, double w){
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return atan2(siny_cosp, cosy_cosp);
}

void IcpBaseSlam::pointcloud2_view(PclCloud &map_cloud, PclCloud &ransac_cloud){
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  sensor_msgs::msg::PointCloud2::SharedPtr ransac_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(map_cloud, *map_msg_ptr);
  pcl::toROSMsg(cloud_add_collor(ransac_cloud, "g"), *ransac_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  ransac_msg_ptr->header.frame_id = "map";
  map_pointcloud2_publisher->publish(*map_msg_ptr);
  ransac_pointcloud2_publisher->publish(*ransac_msg_ptr);

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(estimated_odom.yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  corrent_pose_stamped.header.stamp = this->now();
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = estimated_odom.x;
  corrent_pose_stamped.pose.position.y = estimated_odom.y;
  corrent_pose_stamped.pose.position.z = 0.0;
  corrent_pose_stamped.pose.orientation = quat_msg;

  path.header.stamp = this->now();
  path.header.frame_id = "map";
  path.poses.push_back(corrent_pose_stamped);
  pose_publisher->publish(corrent_pose_stamped);
  path_publisher->publish(path);
}

pcl::PointCloud<pcl::PointXYZRGB> IcpBaseSlam::cloud_add_collor(PclCloud &cloud, char *rgb){
  pcl::PointCloud<pcl::PointXYZRGB> cloud_collor;
  char collor = *rgb;
  for(size_t i=0; i<cloud.points.size(); i++){
    pcl::PointXYZRGB new_point;
    new_point.x = cloud.points[i].x;
    new_point.y = cloud.points[i].y;
    switch(collor){
      case 'r':
        new_point.r = 255;
        break;
      case 'g':
        new_point.g = 255;
        break;
      case 'b':
        new_point.b = 255;
        break;
      }
    cloud_collor.points.push_back(new_point);
  }
  return cloud_collor;
}

void IcpBaseSlam::create_elephant_map(){
  int count=0;
  int count_sum=0;
  //右の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    input_elephant_cloud.points[i].x = double(i)/1000 + map_point_x[0];
    input_elephant_cloud.points[i].y = map_point_y[0];
    count++;
  }
  count_sum=count;
  //後ろの垂木
  for(int i=0; i<=int((map_point_y[3] - map_point_y[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[0];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[0];
    count++;
  }
  count_sum=count;
  //左の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[0];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[3];
    count++;
  }
  count_sum=count;
  //右奥正面向きのフェンス
  for(int i=0; i<=int((map_point_y[1] - map_point_y[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[2];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[0];
    count++;
  }
  count_sum=count;
  //右縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[1];
    count++;
  }
  count_sum=count;
  //正面のフェンス
  for(int i=0; i<=int((map_point_y[2] - map_point_y[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[1];
    count++;
  }
  count_sum=count;
  //左縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[2];
    count++;
  }
  count_sum=count;
  for(int i=0; i<=int((map_point_y[3] - map_point_y[2])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[2];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[2];
    count++;
  }
  ndt.setInputTarget(input_elephant_cloud.makeShared());
}

}
