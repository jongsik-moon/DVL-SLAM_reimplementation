//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(const Config &config)
  : config_(config)
{
  imgSub = nh_.subscribe("/kitti/camera_color_left/image_raw", 1, &Sensor::ImgCb, this);
  pointCloudSub = nh_.subscribe("/kitti/velo/pointcloud", 1, &Sensor::PointCloudCb, this);
  imgPub =  it.advertise("/camera/image", 1);
  input_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);



}

Sensor::~Sensor(){

}

void Sensor::ImgCb(const sensor_msgs::ImageConstPtr& img){
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  input_img_ = cv_ptr->image.clone();
  imgFlag_ = true;
}

void Sensor::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud_);
  lidarFlag_ = true;
}

void Sensor::data2Frame(Frame& frame){
  frame.originalImg_ = input_img_.clone();
  std::cout << (frame.originalCloud_.points.size()) << std::endl;
  if(lidarFlag_) {
    std::cout << "input size" << std::endl;
    std::cout << input_cloud_->points.size() << std::endl;
    pcl::copyPointCloud(*input_cloud_, frame.originalCloud_);
    std::cout << "frame cloud size" << std::endl;
    std::cout << frame.originalCloud_.points.size() << std::endl;

  }
}

void Sensor::publishImg(cv::Mat image){
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  imgPub.publish(msg);
}