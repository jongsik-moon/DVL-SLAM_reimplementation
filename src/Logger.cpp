//
// Created by jongsik on 20. 12. 18..
//

#include "Logger.h"

Logger::Logger()
{
  mapPointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  odometryPointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  imgPub =  it.advertise("/camera/image", 1);
  transPub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_result", 1);
  odometryPointPub = nh_.advertise<sensor_msgs::PointCloud2>("/odometry_result", 1);
  mapPointCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("/map_result", 1);

}

Logger::~Logger()
{

}

void Logger::PushBackOdometryResult(pcl::PointXYZ odometryPoint)
{
  this->odometryPointCloud_->push_back(odometryPoint);
}

void Logger::PushBackMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T)
{
  pcl::PointCloud<pcl::PointXYZRGB> temp;
  pcl::transformPointCloud(mapCloud, temp, T.matrix());

  for(auto point:temp)
  {
    mapPointCloud_->push_back(point);
  }
}

void Logger::SaveMapResult()
{

}

void Logger::SaveOdometryResult()
{

}

void Logger::PublishImg(cv::Mat image){
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, "mono16", image);
  img_bridge.toImageMsg(img_msg);
  imgPub.publish(img_msg);
}

void Logger::PublishTransform(Sophus::SE3f input){

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

void Logger::PublishOdometryPoint(){
  pcl::toROSMsg(*odometryPointCloud_, odometryPC2);
  odometryPC2.header.frame_id = "world";
  odometryPC2.header.stamp = ros::Time::now();
  odometryPointPub.publish(odometryPC2);
}
void Logger::PublishMapPointCloud(){
  pcl::toROSMsg(*mapPointCloud_, mapPC2);
  mapPC2.header.frame_id = "world";
  mapPC2.header.stamp = ros::Time::now();
  mapPointCloudPub.publish(mapPC2);
}