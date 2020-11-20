//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_FRAME_H
#define DVL_SLAM_MODIFY_FRAME_H
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "Config.h"
#include "Datatypes.h"

class Frame {
public:
  typedef std::shared_ptr<Frame> Ptr;

  Frame(Config &config);
  ~Frame();

  cv::Mat PointCloud2Img();

  void setImg();
  void setPointCloud();

  void showImg(cv::Mat& img);
  void saveImg(cv::Mat& img);

  cv::Mat& GetOriginalImg();
  void SetOriginalImg(cv::Mat originalImg);

  pcl::PointCloud<pcl::PointXYZ>& GetOriginalCloud();
  void SetOriginalCloud(pcl::PointCloud<pcl::PointXYZ> originalCloud);

  Eigen::Vector2f PointCloudXyz2Uv(Eigen::Vector3f point);
  cv::Mat pointCloudProjection();

  void JacobianXyz2Uv(Eigen::Vector3f& xyzInF, Matrix2x6& J);

private:
  Config &config_;

  cv::Mat originalImg_;
  pcl::PointCloud<pcl::PointXYZ> originalCloud_;

};


#endif //DVL_SLAM_MODIFY_FRAME_H
