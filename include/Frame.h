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

class Frame {
public:
  Frame(const Config &config);
  ~Frame();

  cv::Mat PointCloud2Img();

  cv::Mat originalImg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud_;

  void setImg();
  void setPointCloud();

  void showImg(cv::Mat& img);
  void saveImg(cv::Mat& img);
  cv::Mat pointCloudProjection(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

private:
  const Config &config_;

};


#endif //DVL_SLAM_MODIFY_FRAME_H
