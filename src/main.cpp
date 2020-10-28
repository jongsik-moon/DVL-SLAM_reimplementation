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

cv_bridge::CvImagePtr cv_ptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcData ;
pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
sensor_msgs::PointCloud2::Ptr fromLaser_cloud;

bool visionFlag = false;
bool lidarFlag = false;


void image_cb(const sensor_msgs::ImageConstPtr& img){
  cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  visionFlag = true;
}

void lidar_cb(const sensor_msgs::LaserScanConstPtr& input){
  laser_geometry::LaserProjection projector;
  projector.projectLaser(*input, *fromLaser_cloud);
  fromLaser_cloud->header.stamp = ros::Time::now();
  pcl::fromROSMsg(*fromLaser_cloud,*temp);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(temp);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0,8);
  pass.filter(*temp);

  lidarFlag = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cali");
  ros::NodeHandle nh;
  pcData = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  temp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  fromLaser_cloud = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

  ros::Subscriber sub_vision = nh.subscribe("/image_raw", 1, image_cb);
  ros::Rate rate(20.0f);

  ros::Subscriber sub_lidar = nh.subscribe("/hokuyo_lidar/info", 1, lidar_cb);
  image_transport::ImageTransport transporter(nh);
  image_transport::Publisher image_pub;
  image_pub = transporter.advertise("/image_calibrated", 1);

  Config config;

  //TODO: tranformation matrix 한번에 되게 하기
  while(ros::ok()){
    if(lidarFlag && visionFlag){
      Eigen::Affine3f transform = pcl::getTransformation(config.delX, config.delY, config.delZ, M_PI, 0, 0);
      pcl::transformPointCloud (*temp, *pcData, transform);
      for(int i=0; i<pcData->points.size(); i++){
        float u = config.fx*(pcData->points[i].y / pcData->points[i].x) + config.cx;
        float v = -config.fy*(pcData->points[i].z / pcData->points[i].x) + config.cy;
        if(u > 0 && u < 688 && v > 0 && v < 516){
          cv::circle(cv_ptr->image, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), 1);
        }

      }
      image_pub.publish(cv_ptr->toImageMsg());
    }
    ros::spinOnce();
  }
}