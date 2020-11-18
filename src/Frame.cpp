//
// Created by jongsik on 20. 10. 30..
//

#include <Frame.h>

Frame::Frame(Config &config)
  : config_(config)
{
  pcPub = nh_.advertise<sensor_msgs::PointCloud2>("publish_cloud", 1);
}

Frame::~Frame(){


}

cv::Mat Frame::pointCloudProjection()
{
  cv::Mat projectedImg = originalImg_.clone();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rectifiedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  transform (0,0) = config_.r11;
  transform (0,1) = config_.r12;
  transform (0,2) = config_.r13;

  transform (1,0) = config_.r21;
  transform (1,1) = config_.r22;
  transform (1,2) = config_.r23;

  transform (2,0) = config_.r31;
  transform (2,1) = config_.r32;
  transform (2,2) = config_.r33;

  transform (0,3) = config_.delX;
  transform (1,3) = config_.delY;
  transform (2,3) = config_.delZ;

  pcl::transformPointCloud (originalCloud_, *transformedCloud, transform);

  Eigen::Matrix4f transformRect = Eigen::Matrix4f::Identity();

  transformRect (0,0) = config_.R11;
  transformRect (0,1) = config_.R12;
  transformRect (0,2) = config_.R13;

  transformRect (1,0) = config_.R21;
  transformRect (1,1) = config_.R22;
  transformRect (1,2) = config_.R23;

  transformRect (2,0) = config_.R31;
  transformRect (2,1) = config_.R32;
  transformRect (2,2) = config_.R33;

  transformRect (0,3) = 0;
  transformRect (1,3) = 0;
  transformRect (2,3) = 0;

  pcl::transformPointCloud (*transformedCloud, *rectifiedCloud, transformRect);

  pcl::toROSMsg(*rectifiedCloud, publish_cloud);
  publish_cloud.header.frame_id = "world";
  publish_cloud.header.stamp = ros::Time::now();
  pcPub.publish(publish_cloud);


  for(int i=0; i<rectifiedCloud->points.size(); i++)
  {
    if(rectifiedCloud->points[i].z > 0.15){


      float U = config_.fx * (rectifiedCloud->points[i].x / rectifiedCloud->points[i].z) + config_.cx;
      float V = config_.fy * (rectifiedCloud->points[i].y / rectifiedCloud->points[i].z) + config_.cy;

      float v_min = 0.15;    float v_max = 10.0;    float dv = v_max - v_min;
      float v = rectifiedCloud->points[i].z;
      float r = 1.0; float g = 1.0; float b = 1.0;
      if (v < v_min)   v = v_min;
      if (v > v_max)   v = v_max;

      if(v < v_min + 0.25*dv) {
        r = 0.0;
        g = 4*(v - v_min) / dv;
      }
      else if (v < (v_min + 0.5 * dv)) {
        r = 0.0;
        b = 1 + 4*(v_min + 0.25 * dv - v) / dv;
      }
      else if (v < (v_min + 0.75 * dv)) {
        r =4 * (v - v_min - 0.5 * dv) / dv;
        b = 0.0;
      }
      else {
        g = 1 + 4*(v_min + 0.75 * dv - v) / dv;
        b = 0.0;
      }

      cv::circle(projectedImg, cv::Point(U, V), 2, cv::Scalar(255*r, 255*g, 255*b), 1);
    }
  }

//  for(int i=0; i<transformedCloud->points.size(); i++)
//  {
//    float u = config_.fx*(transformedCloud->points[i].y / transformedCloud->points[i].x) + config_.cx;
//    float v = -config_.fy*(transformedCloud->points[i].z / transformedCloud->points[i].x) + config_.cy;
//    if(u > 0 && u < 1200 && v > 0 && v < 500){
//      cv::circle(projectedImg, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), 1);
//    }
//  }
  return projectedImg;
}

