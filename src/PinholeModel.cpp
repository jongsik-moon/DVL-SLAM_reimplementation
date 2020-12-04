//
// Created by jongsik on 20. 12. 4..
//
#include "PinholeModel.h"
PinholeModel::PinholeModel(Config& config)
  :config_(config)
{

}

PinholeModel::~PinholeModel()
{

}

Eigen::Vector2f PinholeModel::PointCloudXyz2Uv(Eigen::Vector3f point){
  float U = config_.fx * (point(0) / point(2)) + config_.cx;
  float V = config_.fy * (point(1) / point(2)) + config_.cy;
  Eigen::Vector2f uv(U, V);
  return uv;
}

std::vector<Eigen::Vector2f> PinholeModel::PointCloudXyz2UvVec(const pcl::PointCloud<pcl::PointXYZRGB>& pc){
  std::vector<Eigen::Vector2f> uvSet;
  for(auto point:pc){
    float U = config_.fx * (point.x / point.z) + config_.cx;
    float V = config_.fy * (point.y / point.z) + config_.cy;
    Eigen::Vector2f uv(U, V);
    uvSet.push_back(uv);
    return uvSet;
  }
}
