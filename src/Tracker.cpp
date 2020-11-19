//
// Created by jongsik on 20. 10. 30..
//
#include "Tracker.h"

Tracker::Tracker(Config &config)
  : config_(config)
{

}

Tracker::~Tracker(){

}

void Tracker::Solve(){
  x_ = H_.ldlt().solve(Jres_);
}

void Tracker::UpdatePose(){
  prev_Tji_ = curr_Tji_ * Sophus::SE3f::exp(-x_);
}

void Tracker::CheckVisiblePoints(Frame::Ptr currFrame, Sophus::SE3f& transformation){
  const int border = patch_halfsize_+2;
  cv::Mat& current_img = currFrame->GetOriginalImg();

  const float scale = 1.0f;
  std::vector<bool>::iterator visibility_iter_prev = visible_points_prev_.begin();

  for (auto iter=currFrame->GetOriginalCloud().begin(); iter!=currFrame->GetOriginalCloud().end(); ++iter, ++visibility_iter_prev) {
    Eigen::Vector3f xyz_cur (iter->x, iter->y, iter->z);
    Eigen::Vector3f xyz_prev = transformation.inverse()*xyz_cur;
    Eigen::Vector2f uv_prev;
    uv_prev.noalias() = currFrame->PointCloudXyz2Uv(xyz_prev) * scale;

    const float u_prev_f = uv_prev(0);
    const float v_prev_f = uv_prev(1);
    const int u_prev_i = static_cast<int> (u_prev_f);
    const int v_prev_i = static_cast<int> (v_prev_f);

    if (u_prev_i - border < 0 || u_prev_i + border > current_img.cols || v_prev_i - border < 0 || v_prev_i + border > current_img.rows || xyz_prev(2) <= 0)
      continue;

    *visibility_iter_prev = true;
  }
}

Sophus::SE3f Tracker::trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr keyFrame)
{

}


