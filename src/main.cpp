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

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcData ;
pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
sensor_msgs::PointCloud2::Ptr fromLaser_cloud;

bool visionFlag = false;
bool lidarFlag = false;

float fx = 730.617807179727;
float fy = 731.2502851701612;
float cx = 335.1539179332822;
float cy = 256.15500058538953;


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

  std::string chunkInd;

  std::string str_buf;
  std::fstream fs;

  fs.open("/home/jongsikmoon/cali_ws/src/cv_bridge/src/cali.csv", std::ios::in);
  std::vector<float> cali;
  while(!fs.eof()){
    getline(fs, str_buf, ',');
    float tempfloat = stof(str_buf);
    std::cout << tempfloat << std::endl;
    cali.push_back(tempfloat);
  }


  //TODO: tranformation matrix 한번에 되게 하기
  while(ros::ok()){
    if(lidarFlag && visionFlag){
      Eigen::Affine3f transform = pcl::getTransformation(cali[0], cali[1], cali[2], M_PI, 0, 0);
      pcl::transformPointCloud (*temp, *pcData, transform);
      for(int i=0; i<pcData->points.size(); i++){
        float u = fx*(pcData->points[i].y / pcData->points[i].x) + cx;
        float v = -fy*(pcData->points[i].z / pcData->points[i].x) + cy;
        if(u > 0 && u < 688 && v > 0 && v < 516){
          cv::circle(cv_ptr->image, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), 1);
        }

      }
      image_pub.publish(cv_ptr->toImageMsg());
    }
    ros::spinOnce();
  }
}