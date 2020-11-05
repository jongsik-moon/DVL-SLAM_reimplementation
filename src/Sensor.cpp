//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(){
  imgSub = nh_.subscribe("/image/raw", 1, &Sensor::ImgCb, this);
  pointCloudSub = nh_.subscribe("/image/raw", 1, &Sensor::PointCloudCb, this);

  input_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

Sensor::~Sensor(){

}

void Sensor::ImgCb(const sensor_msgs::ImageConstPtr& img){
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  input_img = cv_ptr->image.clone();
  imgFlag = true;
}

void Sensor::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud);
  lidarFlag = true;
}

void Sensor::data2Frame(Frame& frame){
  frame.originalImg_ = input_img.clone();
  pcl::copyPointCloud(*input_cloud, *frame.originalCloud_);
}