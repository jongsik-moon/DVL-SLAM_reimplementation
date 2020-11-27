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

  inline static void jacobian_xyz2uv(const Eigen::Vector3f& xyzFloat, Matrix2x6& J)
  {
    const float x = xyzFloat[0];
    const float y = xyzFloat[1];
    const float zInv = 1./xyzFloat[2];
    const float zInv2 = zInv*zInv;

    J(0,0) = -zInv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*zInv2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*zInv;             // y/z


    J(1,0) = 0.0;                 // 0
    J(1,1) = -zInv;              // -1/z
    J(1,2) = y*zInv2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*zInv;            // x/z
  }
private:
  Config &config_;

  cv::Mat originalImg_;
  pcl::PointCloud<pcl::PointXYZ> originalCloud_;

};


#endif //DVL_SLAM_MODIFY_FRAME_H
