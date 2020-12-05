//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(Config &config)
  : config_(config)
{
  std::string imgTopic;
  std::string pcTopic;
  if(config_.isIndoor)
  {
    imgTopic = "/image_raw";
    pcTopic = "/velodyne_points";
  }
  else if(config_.isKitti)
  {
    imgTopic = "/kitti/camera_color_left/image_raw";
    pcTopic = "/kitti/velo/pointcloud";
  }

  imgSub = nh_.subscribe(imgTopic, 1, &Sensor::ImgCb, this);
  pointCloudSub = nh_.subscribe(pcTopic, 1, &Sensor::PointCloudCb, this);
  imgPub =  it.advertise("/camera/image", 1);
  transPub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_result", 1);
  input_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  lidarFlag_ = false;
  imgFlag_ = false;

  minZ = 0.15f;
  maxZ = 50.0f;
}

Sensor::~Sensor(){

}

void Sensor::ImgCb(const sensor_msgs::ImageConstPtr& img){
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  input_img_ = cv_ptr->image.clone();
  imgFlag_ = true;
}

void Sensor::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *input_cloud_);
  lidarFlag_ = true;
}

void Sensor::data2Frame(Frame& frame){
  frame.SetOriginalImg(input_img_);
  if(lidarFlag_) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectifiedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


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

    pcl::transformPointCloud (*input_cloud_, *transformedCloud, transform);

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

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(rectifiedCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minZ, maxZ);
    pass.filter(*rectifiedCloud);


    frame.SetOriginalCloud(*rectifiedCloud);
  }

}

void Sensor::publishImg(cv::Mat image){
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, "mono16", image);
  img_bridge.toImageMsg(img_msg);
  imgPub.publish(img_msg);
}

void Sensor::publishTransform(Sophus::SE3f input){

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

bool Sensor::IsLidarSubscribed(){ return lidarFlag_; }

bool Sensor::IsVisionSubscribed(){ return imgFlag_; }