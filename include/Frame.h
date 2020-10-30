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

private:

  cv::Mat originalImg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr intputCloud_;

  const Config &config_;


};


#endif //DVL_SLAM_MODIFY_FRAME_H
