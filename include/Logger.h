//
// Created by jongsik on 20. 12. 18..
//

#ifndef DVL_SLAM_MODIFY_LOGGER_H
#define DVL_SLAM_MODIFY_LOGGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/transforms.h>

class Logger{
public:
  Logger();
  ~Logger();

  void PushBackOdometryResult(pcl::PointXYZ odometryPoint);
  void PushBackMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T);

  void SaveOdometryResult();
  void SaveMapResult();

  void PublishImg(cv::Mat image);
  void PublishTransform(Sophus::SE3f input);

  void PublishOdometryPoint();
  void PublishMapPointCloud();


private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr odometryPointCloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapPointCloud_;

  sensor_msgs::PointCloud2 odometryPC2;
  sensor_msgs::PointCloud2 mapPC2;

  ros::NodeHandle nh_;

  ros::Publisher transPub;
  ros::Publisher odometryPointPub;
  ros::Publisher mapPointCloudPub;

  image_transport::ImageTransport it = image_transport::ImageTransport(ros::NodeHandle());
  image_transport::Publisher imgPub;
};



#endif //DVL_SLAM_MODIFY_LOGGER_H
