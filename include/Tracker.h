//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_TRACKER_H
#define DVL_SLAM_MODIFY_TRACKER_H

#include "Config.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <sophus/se3.hpp>

class Tracker{
public:

  Tracker(Config &config);
  ~Tracker();

  Config& config_;

  Sophus::SE3f trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr keyFrame);

};


#endif //DVL_SLAM_MODIFY_TRACKER_H
