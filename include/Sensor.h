//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_SENSOR_H
#define DVL_SLAM_MODIFY_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

class Sensor{
public:
  Sensor();
  ~Sensor();

  void ImgCb(const sensor_msgs::ImageConstPtr& img);
  void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);

private:
  ros::Subscriber imgSub;
  ros::Subscriber pointCloudSub;

  ros::NodeHandle nh_;

};

#endif //DVL_SLAM_MODIFY_SENSOR_H
