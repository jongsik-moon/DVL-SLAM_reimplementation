//
// Created by jongsik on 20. 10. 30..
//

#include <Frame.h>

Frame::Frame(const Config &config)
  : config_(config)
{

}

Frame::~Frame(){


}

cv::Mat Frame::PointCloud2Img()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cv::Mat projectedImg = originalImg_.clone();

  Eigen::Affine3f transform = pcl::getTransformation(config_.delX, config_.delY, config_.delZ, M_PI, 0, 0);
  pcl::transformPointCloud (*intputCloud_, *temp, transform);
  for(int i=0; i<temp->points.size(); i++){
    float u = config_.fx*(temp->points[i].y / temp->points[i].x) + config_.cx;
    float v = -config_.fy*(temp->points[i].z / temp->points[i].x) + config_.cy;
    if(u > 0 && u < 688 && v > 0 && v < 516){
      cv::circle(projectedImg, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), 1);
    }
  }
  return projectedImg;
}
