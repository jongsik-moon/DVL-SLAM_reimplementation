//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_TRACKER_H
#define DVL_SLAM_MODIFY_TRACKER_H

#include "Config.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <sophus/se3.hpp>
#include "Datatypes.h"

class Tracker{
public:

  Tracker(Config &config);
  ~Tracker();

  Config& config_;

  void Solve();
  void UpdatePose();
  void CheckVisiblePoints(Frame::Ptr currFrame, Sophus::SE3f& transformation);
  Sophus::SE3f trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr keyFrame);

private:
  int patch_halfsize_ = 2;

  Vector6 x_;
  Vector6 Jres_;
  Matrix6x6 H_;
  Matrix6x6 prev_H_;

  Sophus::SE3f prev_Tji_;
  Sophus::SE3f curr_Tji_;

  std::vector<bool> visible_points_;
  std::vector<bool> visible_points_prev_;

};


#endif //DVL_SLAM_MODIFY_TRACKER_H
