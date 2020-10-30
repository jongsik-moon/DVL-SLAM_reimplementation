//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(){
  imgSub = nh_.subscribe("/image/raw", 1, &Sensor::ImgCb, this);
  pointCloudSub = nh_.subscribe("/image/raw", 1, &Sensor::PointCloudCb, this);

}

Sensor::~Sensor(){

}

void Sensor::ImgCb(const sensor_msgs::ImageConstPtr& img){

}

void Sensor::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){

}
