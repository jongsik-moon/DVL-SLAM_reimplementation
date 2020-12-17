//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(Config& config)
: config_(config)
{
  lidarFlag_ = false;
  imgFlag_ = false;

  minZ = 0.15f;
  maxZ = 50.0f;

  imgPub =  it.advertise("/camera/image", 1);
  transPub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_result", 1);

  inputCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

Sensor::~Sensor(){
}

void Sensor::publishImg(cv::Mat image){
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, "mono16", image);
  img_bridge.toImageMsg(img_msg);
  imgPub.publish(img_msg);
}

void Sensor::publishTransform(Sophus::SE3f input){

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = input.translation().x();
  msg.pose.position.y = input.translation().y();
  msg.pose.position.z = input.translation().z();
  msg.pose.orientation.x = input.unit_quaternion().x();
  msg.pose.orientation.y = input.unit_quaternion().y();
  msg.pose.orientation.z = input.unit_quaternion().z();
  msg.pose.orientation.w = input.unit_quaternion().w();
  msg.header.frame_id = "world";

//  geometry_msgs::Transform msg;
//  msg.translation.x = input.translation().x();
//  msg.translation.y = input.translation().y();
//  msg.translation.y = input.translation().y();
//  msg.rotation.x = input.unit_quaternion().x();
//  msg.rotation.x = input.unit_quaternion().y();
//  msg.rotation.x = input.unit_quaternion().z();
//  msg.rotation.x = input.unit_quaternion().w();

  transPub.publish(msg);
}

bool Sensor::IsLidarSubscribed(){ return lidarFlag_; }

bool Sensor::IsVisionSubscribed(){ return imgFlag_; }