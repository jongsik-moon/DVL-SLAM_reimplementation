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

  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;

  static const int pattern_length_ = 8;
  int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:

  Tracker(Config &config);
  ~Tracker();

  Config& config_;

  void Solve();
  void UpdatePose();
  float HuberWeight(const float res);
  void CheckVisiblePointsInPrevFrame(Frame::Ptr currFrame, Sophus::SE3f& transformation);
  void PrecomputeReferencePatterns();
  double ComputeResidualsPatterns(Sophus::SE3f& transformation);
  Sophus::SE3f trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr keyFrame);

private:
  Frame::Ptr currentFrame_;
  KeyFrame::Ptr referenceFrame_;

  float huberK_;

  Vector6 x_;
  Vector6 Jres_;
  Matrix6x6 H_;
  Matrix6x6 prev_H_;

  Sophus::SE3f prev_Tji_;
  Sophus::SE3f curr_Tji_;

  std::vector<bool> visiblePoints_;
  std::vector<bool> visiblePointsInCur_;
  std::vector<bool> visiblePointsPrev_;

  std::vector<float> errors_;
  cv::Mat refPatchBuf_;

  int currentLevel_;
  bool isSetRefPatch_;
  size_t nMeasurement_;

  Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobianBuf_;
};


#endif //DVL_SLAM_MODIFY_TRACKER_H
