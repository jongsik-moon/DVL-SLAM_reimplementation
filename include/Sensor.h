//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_SENSOR_H
#define DVL_SLAM_MODIFY_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Frame.h"

class Sensor{
public:
  Sensor();
  ~Sensor();

  void ImgCb(const sensor_msgs::ImageConstPtr& img);
  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
  void data2Frame(Frame& frame);

private:
  ros::Subscriber imgSub;
  ros::Subscriber pointCloudSub;

  ros::NodeHandle nh_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
  cv::Mat input_img;

  bool lidarFlag;
  bool imgFlag;

};

#endif //DVL_SLAM_MODIFY_SENSOR_H
