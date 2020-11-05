#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/registration/transforms.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <Config.h>

using namespace std;



int main(int argc, char** argv){
  ros::init(argc, argv, "dvl_reimpl");
  ros::NodeHandle nh;


  ros::Rate rate(20.0f);

  image_transport::ImageTransport transporter(nh);
  image_transport::Publisher image_pub;
  image_pub = transporter.advertise("/image_calibrated", 1);

  //TODO: tranformation matrix 한번에 되게 하기
  while(ros::ok()){
    ros::spinOnce();
  }
}