//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_SYSTEM_H
#define DVL_SLAM_MODIFY_SYSTEM_H

#include "Config.h"
#include "Frame.h"
#include "GraphOptimizer.h"
#include "KeyFrame.h"
#include "Sensor.h"
#include "Tracker.h"
#include <sophus/se3.hpp>

class System{
public:
  System(Config& config_);
  ~System();

  void Run();

private:
  Config& config_;
  GraphOptimizer graphOptimizer_;
  Sensor sensor_;
  Tracker tracker_;

  bool initialized_;

  Sophus::SE3f Tij_;
  Sophus::SE3f Tji_;
  Sophus::SE3f dTji_;

  std::vector<Frame::Ptr> frameDB_;
  std::vector<KeyFrame::Ptr> keyFrameDB_;
};


#endif //DVL_SLAM_MODIFY_SYSTEM_H
