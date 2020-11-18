//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_KEYFRAME_H
#define DVL_SLAM_MODIFY_KEYFRAME_H

#include "Config.h"
#include "Frame.h"

class KeyFrame{
public:
  typedef std::shared_ptr<KeyFrame> Ptr;

  KeyFrame(Config &config, Frame::Ptr frame);
  ~KeyFrame();

  Config& config_;
  Frame::Ptr frame;

};

#endif //DVL_SLAM_MODIFY_KEYFRAME_H
