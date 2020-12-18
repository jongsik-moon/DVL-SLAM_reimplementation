//
// Created by jongsik on 20. 12. 18..
//

#include "Logger.h"

Logger::Logger(Config& config)
  : pinholeModel_(config)
  , config_(config)
{
  mapPointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  odometryPointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  imgPub =  it.advertise("/camera/image", 1);
  transPub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_result", 1);
  odometryPointPub = nh_.advertise<sensor_msgs::PointCloud2>("/odometry_result", 1);
  mapPointCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("/map_result", 1);

  voxelSize_ = config_.loggerConfig.voxelSize;
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
  int border = config_.trackerConfig.border;

  pcl::PointCloud<pcl::PointXYZRGB> temp;
  pcl::transformPointCloud(mapCloud, temp, T.matrix());

  std::vector<Eigen::Vector2f> uvSet = pinholeModel_.PointCloudXyz2UvVec(temp, 1);
  auto pointCloudIter = temp.begin();
  for(auto uvIter=uvSet.begin(); uvIter!=uvSet.end(); ++uvIter, ++pointCloudIter)
  {
    Eigen::Vector2f uv = *uvIter;

    const float uFloat = uv(0);
    const float vFloat = uv(1);
    const int uInt = static_cast<int> (uFloat);
    const int vInt = static_cast<int> (vFloat);

    if(pointCloudIter->r != 0)
    {
      mapPointCloud_->push_back(*pointCloudIter);
    }


  }

//  voxelFilter_.setInputCloud(mapPointCloud_);
//  voxelFilter_.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
//  voxelFilter_.filter(*mapPointCloud_);
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
  img_bridge = cv_bridge::CvImage(header, "bgr8", image);
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