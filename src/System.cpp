//
// Created by jongsik on 20. 10. 30..
//

#include "System.h"

System::System(Config& config)
  : config_(config),
    graphOptimizer_(config_),
    sensor_(config_),
    tracker_(config_)
{
  initialized_ = false;

}

System::~System(){


}


void System::Run(){
  Frame::Ptr currFrame (new Frame(config_));
  sensor_.data2Frame(*currFrame);
  sensor_.publishImg(currFrame->pointCloudProjection());

  if(!initialized_){
    Eigen::Matrix3f rot;
    rot << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;
    Eigen::Vector3f twc(0, 0, 0);

    Sophus::SE3f sophus_twc(rot, twc);

    initialized_ = true;

    KeyFrame::Ptr keyFrame(new KeyFrame(config_, currFrame));
    frameDB_.push_back(currFrame);
    keyFrameDB_.push_back(keyFrame);

  }
  else{
    KeyFrame::Ptr lastKeyFrame = *keyFrameDB_.end();

    tracker_.trackFrame2Frame(currFrame, lastKeyFrame);

  }

}