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

class System{
public:
  System(const Config& config_);
  ~System();

  void Run();

private:
  const Config& config_;
  Frame frame_;
  GraphOptimizer graphOptimizer_;
  KeyFrame keyFrame_;
  Sensor sensor_;

};


#endif //DVL_SLAM_MODIFY_SYSTEM_H
