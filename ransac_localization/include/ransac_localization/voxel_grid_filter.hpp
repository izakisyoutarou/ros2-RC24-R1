#include <vector>
#include <iostream>
#include "config.hpp"

typedef pair<int, int> IntPair;

class VoxelGridFilter{
public:
  void setup(const double &voxel_size){
    voxel_size_ = voxel_size;
  }

  vector<LaserPoint> apply_voxel_grid_filter(const vector<LaserPoint>& input_points) {
    std::map<IntPair, std::vector<LaserPoint>> voxel_grid;
    vector<LaserPoint> filtered_points;
    // 各点を対応するセルに割り当て
    for (const auto& point : input_points) {
        int x_voxel = static_cast<int>(point.x / voxel_size_);
        int y_voxel = static_cast<int>(point.y / voxel_size_);
        IntPair voxel_index(x_voxel, y_voxel);
        voxel_grid[voxel_index].push_back(point);
    }

    // 各セル内の点群を代表点に置き換え
    for (const auto& voxel : voxel_grid) {
      const auto& points = voxel.second;
      LaserPoint centroid{0,0};
      for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
      }
      centroid.x /= points.size();
      centroid.y /= points.size();
      filtered_points.push_back(centroid);
    }

    return filtered_points;
  }

private:
  double voxel_size_;
};
