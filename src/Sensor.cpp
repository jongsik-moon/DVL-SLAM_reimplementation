//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(Config &config)
  : config_(config)
{
  std::string imgTopic;
  std::string pcTopic;
  if(config_.isIndoor)
  {
    imgTopic = "/image_raw";
    pcTopic = "/velodyne_points";
  }
  else if(config_.isKitti)
  {
    imgTopic = "/kitti/camera_color_left/image_raw";
    pcTopic = "/kitti/velo/pointcloud";
  }

  imgSub = nh_.subscribe(imgTopic, 1, &Sensor::ImgCb, this);
  pointCloudSub = nh_.subscribe(pcTopic, 1, &Sensor::PointCloudCb, this);
  imgPub =  it.advertise("/camera/image", 1);
  input_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  lidarFlag_ = false;
  imgFlag_ = false;
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
  if(lidarFlag_) {
    std::cout << input_cloud_->points.size() << std::endl;
    pcl::copyPointCloud(*input_cloud_, frame.originalCloud_);
    std::cout << frame.originalCloud_.points.size() << std::endl;
  }
}

void Sensor::publishImg(cv::Mat image){
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, "bgr8", image);
  img_bridge.toImageMsg(img_msg);
  imgPub.publish(img_msg);
}