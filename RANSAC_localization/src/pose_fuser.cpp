#include "RANSAC_localization/pose_fuser.hpp"


void PoseFuser::setup(const double laser_weight, const double odom_weight){
  laser_weight_ = laser_weight;
  odom_weight_ = odom_weight;
}

void PoseFuser::init(){
  current_points.clear();
  reference_points.clear();
}

Vector3d PoseFuser::fuse_pose(const Vector3d &laser_estimated, const Vector3d &scan_odom_motion, const Vector3d &current_scan_odom, const double dt_scan, const vector<LaserPoint> &src_points, const vector<LaserPoint> &global_points){
  init();
  if(global_points.size()==0) return current_scan_odom;
  NormalVector normal_vector = find_correspondence(src_points, global_points, current_points, reference_points);
  Matrix3d laser_cov = laser_weight_*calc_laser_cov(laser_estimated, current_points, reference_points, normal_vector);
  Matrix3d scan_odom_motion_cov = odom_weight_*calculate_motion_cov(scan_odom_motion, dt_scan);  // オドメトリで得た移動量の共分散

  Matrix3d rotate_scan_odom_motion_cov = rotate_cov(laser_estimated, scan_odom_motion_cov);
  Vector3d estimated;
  Matrix3d fused_cov;               // センサ融合後の共分散
  fuse(laser_estimated, laser_cov, current_scan_odom, rotate_scan_odom_motion_cov, estimated, fused_cov);  // 2つの正規分布の融合
  return estimated;
}

NormalVector PoseFuser::find_correspondence(const vector<LaserPoint> &src_points, const vector<LaserPoint> &global_points, vector<CorrespondLaserPoint> &current_points, vector<CorrespondLaserPoint> &reference_points){
  CorrespondLaserPoint global;
  CorrespondLaserPoint current;
  NormalVector normal_vector;
  double sum_x=0.0;
  double sum_y=0.0;
  for(size_t i=0; i<global_points.size(); i++){
    current.x = src_points[i].x;
    current.y = src_points[i].y;
    global.x = global_points[i].x;
    global.y = global_points[i].y;
    CorrespondLaserPoint closest_reference = find_closest_vertical_point(global);
    sum_x+=closest_reference.nx;
    sum_y+=closest_reference.ny;
    current_points.push_back(current);
    reference_points.push_back(closest_reference);
  }
  const double L = sqrt(sum_x*sum_x + sum_y*sum_y);
  normal_vector.normalize_x = sum_x / L; // 平均（正規化）
  normal_vector.normalize_y = sum_y / L;
  return normal_vector;
}

CorrespondLaserPoint PoseFuser::find_closest_vertical_point(CorrespondLaserPoint global){
  CorrespondLaserPoint closest;
  CorrespondLaserPoint vertical_distance;
  double distance_min = 100.0;
  for(int i=0; i<4; i++){
    vertical_distance.x = fabs(map_point_x[i] - global.x);
    vertical_distance.y = fabs(map_point_y[i] - global.y);
    if(vertical_distance.x < distance_min){
      distance_min = vertical_distance.x;
      closest.x = map_point_x[i];
      closest.y = global.y;
    }
    if(vertical_distance.y < distance_min){
      distance_min = vertical_distance.y;
      closest.x = global.x;
      closest.y = map_point_y[i];
    }
  }
  if(closest.x==map_point_x[0] || closest.x==map_point_x[1] || closest.x==map_point_x[2] || closest.x==map_point_x[3]){
    closest.nx=1.0;
    closest.ny=0.0;
  }
  else{
    closest.nx=0.0;
    closest.ny=1.0;
  }
  return closest;
}

Matrix3d PoseFuser::calc_laser_cov(const Vector3d &laser_estimated, vector<CorrespondLaserPoint> &current_points, vector<CorrespondLaserPoint> &reference_points, NormalVector normal_vector){
  const double dd = 1e-5;  //数値微分の刻み
  vector<double> Jx; //ヤコビ行列のxの列
  vector<double> Jy; //ヤコビ行列のyの列
  vector<double> Jyaw; //ヤコビ行列のyawの列

  for(size_t i=0; i<current_points.size(); i++){
    double vertical_distance   = calculate_vertical_distance(current_points[i], reference_points[i], laser_estimated[0],    laser_estimated[1],    laser_estimated[2], normal_vector);
    double vertical_distance_x = calculate_vertical_distance(current_points[i], reference_points[i], laser_estimated[0]+dd, laser_estimated[1],    laser_estimated[2], normal_vector);
    double vertical_distance_y = calculate_vertical_distance(current_points[i], reference_points[i], laser_estimated[0],    laser_estimated[1]+dd, laser_estimated[2], normal_vector);
    double vertical_distance_yaw = calculate_vertical_distance(current_points[i], reference_points[i], laser_estimated[0],    laser_estimated[1], laser_estimated[2] + dd, normal_vector);
    Jx.push_back((vertical_distance_x - vertical_distance) / dd);
    Jy.push_back((vertical_distance_y - vertical_distance) / dd);
    Jyaw.push_back((vertical_distance_yaw - vertical_distance) / dd);
  }
  // ヘッセ行列の近似J^TJの計算
  Matrix3d hes = Matrix3d::Zero(3,3);          // 近似ヘッセ行列。0で初期化
  for (size_t i=0; i<Jx.size(); i++) {
    hes(0,0) += Jx[i]*Jx[i];
    hes(0,1) += Jx[i]*Jy[i];
    hes(0,2) += Jx[i]*Jyaw[i];
    hes(1,1) += Jy[i]*Jy[i];
    hes(1,2) += Jy[i]*Jyaw[i];
    hes(2,2) += Jyaw[i]*Jyaw[i];
  }
  // J^TJが対称行列であることを利用
  hes(1,0) = hes(0,1);
  hes(2,0) = hes(0,2);
  hes(2,1) = hes(1,2);
  const double esp = 1e-6;
  hes += esp * Matrix3d::Identity();   //行列の要素が0になり、逆行列が求まらない場合の対処
  return svdInverse(hes);
}

double PoseFuser::calculate_vertical_distance(const CorrespondLaserPoint current, const CorrespondLaserPoint reference, double x, double y, double yaw, NormalVector normal_vector){
  const double x_ = cos(yaw)*current.x - sin(yaw)*current.y + x;                     // clpを推定位置で座標変換
  const double y_ = sin(yaw)*current.x + cos(yaw)*current.y + y;
  return (x_-reference.x)*normal_vector.normalize_x + (y_-reference.y)*normal_vector.normalize_y;
}

Matrix3d PoseFuser::calculate_motion_cov(const Vector3d &scan_odom_motion, const double dt_scan){
  double vt = sqrt(scan_odom_motion[0]*scan_odom_motion[0] + scan_odom_motion[1]*scan_odom_motion[1]) / dt_scan;
  double wt = abs(scan_odom_motion[2]/dt_scan);
  const double thre = 0.1;                   // 静止時にlidarを信頼しすぎないための下限値
  if (vt < thre) vt = thre;
  if (wt < thre) wt = thre;
  Matrix3d C1;
  C1.setZero();
  C1(0,0) = 1/(vt*vt);                 // 並進成分x
  C1(1,1) = 1/(vt*vt);                 // 並進成分y
  C1(2,2) = 1/(wt*wt);                 // 回転成分

  return C1;
}

Matrix3d PoseFuser::rotate_cov(const Vector3d &laser_estimated, Matrix3d &scan_odom_motion_cov){
  const double cs = cos(laser_estimated[2]);            // poseの回転成分thによるcos
  const double sn = sin(laser_estimated[2]);
  Matrix3d J;                            // 回転のヤコビ行列
  J << cs, -sn,  0,
       sn,  cs,  0,
        0,   0,  1;
  Matrix3d JT = J.transpose();
  return J*scan_odom_motion_cov*JT;  // 回転変換
}

double PoseFuser::fuse(const Vector3d &laser_estimated, const Matrix3d &laser_cov, const Vector3d &current_scan_odom, const Matrix3d &rotate_scan_odom_motion_cov, Vector3d &estimated, Matrix3d &fused_cov){
  // 共分散行列の融合
  Matrix3d IC1 = svdInverse(laser_cov);
  Matrix3d IC2 = svdInverse(rotate_scan_odom_motion_cov);
  Matrix3d IC = IC1 + IC2;
  fused_cov = svdInverse(IC);

  //角度を連続に保つ
  Vector3d laser_estimated_ = laser_estimated;
  double da = current_scan_odom[2] - laser_estimated[2];
  if (da > M_PI) laser_estimated_[2] += 2*M_PI;
  else if (da < -M_PI) laser_estimated_[2] -= 2*M_PI;

  // 平均の融合
  Vector3d nu1 = IC1*laser_estimated;
  Vector3d nu2 = IC2*current_scan_odom;
  Vector3d nu3 = nu1 + nu2;
  estimated = fused_cov*nu3;
  estimated[2] = normalize_yaw(estimated[2]);
}


// SVDを用いた逆行列計算
Matrix3d PoseFuser::svdInverse(const Matrix3d &A) {
  size_t m = A.rows();
  size_t n = A.cols();
  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
  MatrixXd eU = svd.matrixU();
  MatrixXd eV = svd.matrixV();
  VectorXd eS = svd.singularValues();
  MatrixXd M1(m, n);
  for (size_t i=0; i<n; i++) {
    for (size_t j=0; j<n; j++) {
      M1(i,j) = eU(j,i)/eS[i];
    }
  }
  Matrix3d IA;
  for (size_t i=0; i<n; i++) {
    for (size_t j=0; j<n; j++) {
      IA(i,j) = 0;
      for (size_t k=0; k<n; k++)
        IA(i,j) += eV(i,k)*M1(k,j);
    }
  }
  return(IA);
}
