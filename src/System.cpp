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

  keyFrameDB_.reset(new KeyFrameDB());
  frameDB_.reset(new FrameDB());
}

System::~System(){

}

void System::Run(){
  Frame::Ptr currFrame (new Frame(config_));
  if(!sensor_.IsLidarSubscribed() || !sensor_.IsVisionSubscribed()){
    std::cout << "[System] No sensor data subscribed" << std::endl;
    return;
  }
  sensor_.data2Frame(*currFrame);
//  sensor_.publishImg(currFrame->GetPyramidImg(1));

  std::cout << "[System] frame class got data from sensor class" << std::endl;

  if(!initialized_){
    Eigen::Matrix3f rot;
    rot << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;
    Eigen::Vector3f twc(0, 0, 0);

    Sophus::SE3f initialTwc(rot, twc);
    currFrame->SetTwc(initialTwc);

    KeyFrame::Ptr keyFrame(new KeyFrame(config_, currFrame));
    frameDB_->Add(currFrame);
    keyFrameDB_->Add(keyFrame);

    initialized_ = true;
    std::cout << "[System] Initialized" << std::endl;
    return;
  }
  else{

    std::cout << "[System] Find KeyFrame from KeyFrame DB" << std::endl;

    KeyFrame::Ptr lastKeyFrame = keyFrameDB_->LatestKeyframe();

    Sophus::SE3f prevTji = Tji_;
    tracker_.trackFrame2Frame(currFrame, lastKeyFrame, Tji_);

    dTji_ = Tji_ * prevTji.inverse();
    Tij_ = Tji_.inverse();

    std::cout << "[System] Tracking Finished" << std::endl;

    Sophus::SE3f Twc = lastKeyFrame->frame->GetTwc();
    currFrame->SetTwc(Twc * Tij_);

    float ratio_threshold = 1.0;
    std::cout << "[System] Find Keyframe" << std::endl;

    KeyFrame::Ptr currentKeyframe(new KeyFrame(config_, currFrame));

    float visible_ratio1 = lastKeyFrame->GetVisibleRatio(currentKeyframe);
    float visible_ratio2 = currentKeyframe->GetVisibleRatio(lastKeyFrame);

    bool is_keyframe = (visible_ratio1 < ratio_threshold ? true : false) || ((visible_ratio2 < ratio_threshold ? true : false));

    std::cout << "[System] KeyFrame Decision calculated" << std::endl;

    if(is_keyframe)
    {
      keyFrameDB_->Add(currentKeyframe);
      std::cout << "[System] Add KeyFrame" << std::endl;
    }
    std::cout << "[System] Finished" << std::endl;

    sensor_.publishTransform(Twc * Tij_);
  }

}