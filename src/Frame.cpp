//
// Created by jongsik on 20. 10. 30..
//

#include <Frame.h>

Frame::Frame(Config &config)
  : config_(config)
{
}

Frame::~Frame(){


}


cv::Mat& Frame::GetOriginalImg(){ return originalImg_; }

void Frame::SetOriginalImg(cv::Mat originalImg){ this->originalImg_ = originalImg; }

pcl::PointCloud<pcl::PointXYZRGB>& Frame::GetOriginalCloud(){ return originalCloud_; }

void Frame::SetOriginalCloud(pcl::PointCloud<pcl::PointXYZRGB> originalCloud){ this->originalCloud_ = originalCloud; }

Sophus::SE3f Frame::GetTwc(){ return Twc_; }

void Frame::SetTwc(Sophus::SE3f Twc){ this->Twc_ = Twc; }

cv::Mat Frame::pointCloudProjection()
{
  cv::Mat projectedImg = originalImg_.clone();

  for(int i=0; i<originalCloud_.points.size(); i++)
  {
    float U = config_.fx * (originalCloud_.points[i].x / originalCloud_.points[i].z) + config_.cx;
    float V = config_.fy * (originalCloud_.points[i].y / originalCloud_.points[i].z) + config_.cy;

    float v_min = 0.15;    float v_max = 50.0;    float dv = v_max - v_min;
    float v = originalCloud_.points[i].z;
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
  return projectedImg;
}

