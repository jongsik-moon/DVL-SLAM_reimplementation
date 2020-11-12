//
// Created by jongsik on 20. 10. 30..
//

#include <Frame.h>

Frame::Frame(const Config &config)
  : config_(config)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

}

Frame::~Frame(){


}

cv::Mat Frame::pointCloudProjection(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
  cv::Mat projectedImg = img.clone();

  Eigen::Affine3f transform = pcl::getTransformation(config_.delX, config_.delY, config_.delZ, M_PI, 0, 0);
  pcl::transformPointCloud (*originalCloud_, *pointCloud, transform);
  for(int i=0; i<pointCloud->points.size(); i++){
    float u = config_.fx*(pointCloud->points[i].y / pointCloud->points[i].x) + config_.cx;
    float v = -config_.fy*(pointCloud->points[i].z / pointCloud->points[i].x) + config_.cy;
    if(u > 0 && u < 688 && v > 0 && v < 516){
      cv::circle(projectedImg, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), 1);
    }
  }
  return projectedImg;
}

