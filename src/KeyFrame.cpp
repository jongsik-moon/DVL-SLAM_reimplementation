//
// Created by jongsik on 20. 11. 11..
//
#include "KeyFrame.h"

KeyFrame::KeyFrame(Config &config, Frame::Ptr frame)
  :config_(config),
  frame(frame)
{
  pinholeModel_ = std::static_pointer_cast<PinholeModel> (pinholeModel_);
}

KeyFrame::~KeyFrame()
{

}

float KeyFrame::GetVisibleRatio (const KeyFrame::Ptr keyframe)
{
  Sophus::SE3f Tij = frame->GetTwc().inverse() * keyframe->frame->GetTwc();

  int patch_halfsize_ = 2;
  const int border = patch_halfsize_+2;
  int current_level = 0;

  cv::Mat& current_img = frame->GetOriginalImg();


  int visible_points = 0;

  for (auto iter=keyframe->frame->GetOriginalCloud().begin(); iter!=keyframe->frame->GetOriginalCloud().end(); ++iter) {
    Eigen::Vector3f xyz_cur (iter->x, iter->y, iter->z);
    Eigen::Vector3f xyz_prev = Tij*xyz_cur;
    Eigen::Vector2f uv_prev;
    uv_prev.noalias() = pinholeModel_->PointCloudXyz2Uv(xyz_prev);

    const float u_prev_f = uv_prev(0);
    const float v_prev_f = uv_prev(1);
    const int u_prev_i = static_cast<int> (u_prev_f);
    const int v_prev_i = static_cast<int> (v_prev_f);

    if (u_prev_i - border < 0 || u_prev_i + border > current_img.cols || v_prev_i - border < 0 || v_prev_i + border > current_img.rows || xyz_prev(2) <= 0)
      continue;

    visible_points++;

  }

  return static_cast<float> (visible_points) / static_cast<float> (keyframe->frame->GetOriginalCloud().size());
}