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
#include <image_transport/image_transport.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

class Sensor{
public:
  Sensor(Config &config);
  ~Sensor();

  void ImgCb(const sensor_msgs::ImageConstPtr& img);
  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
  void data2Frame(Frame& frame);

  void publishImg(cv::Mat image);

private:
  Config& config_;

  ros::NodeHandle nh_;

  ros::Subscriber imgSub;
  ros::Subscriber pointCloudSub;

  image_transport::ImageTransport it = image_transport::ImageTransport(ros::NodeHandle());
  image_transport::Publisher imgPub;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
  cv::Mat input_img_;


  bool lidarFlag_;
  bool imgFlag_;

  float minZ;
  float maxZ;

};

#endif //DVL_SLAM_MODIFY_SENSOR_H
