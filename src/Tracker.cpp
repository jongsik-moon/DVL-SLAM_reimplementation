//
// Created by jongsik on 20. 10. 30..
//
#include "Tracker.h"

Tracker::Tracker(Config &config)
  : config_(config)
{
  huberK_ = 1.345;
  maxIteration = 100;
  residual_ = 0;
  eps_ = 0.000001;

}

Tracker::~Tracker(){

}

bool Tracker::Solve(){
  x_ = H_.ldlt().solve(Jres_);
  if ( ((x_ - x_).array() == (x_ - x_).array()).all() )
    return true;
  return false;
}

void Tracker::UpdatePose(const Sophus::SE3f& old_Tji, Sophus::SE3f& Tji){
  Tji = old_Tji * Sophus::SE3f::exp(-x_);
}

void Tracker::Optimize(Sophus::SE3f& Tji){
  Sophus::SE3f oldTji = Tji;
  Matrix6x6 oldH = Matrix6x6::Identity();

  stop_ = false;

  for (int i=0; i<maxIteration; i++){
    H_.setZero();
    Jres_.setZero();

    double newResidual = ComputeResidualsPatterns(Tji);

    if(!Solve()){
      stop_ = true;
    }
    if(i > 0 && newResidual > residual_ || stop_){
      Tji = oldTji;
      H_ = oldH;
      break;
    }
    Sophus::SE3f newTji;
    UpdatePose(Tji, newTji);
    oldTji = Tji;
    oldH = H_;
    Tji = newTji;

    residual_ =newResidual;
    if(NormMax(x_) < eps_){
      status_ = true;

      if ( ((x_ - x_).array() != (x_ - x_).array()).all() )
        status_ = false;

      break;
    }
  }
}

float Tracker::HuberWeight(const float res){
  float tAbs = fabsf(res);
  float sig = 5.0;
  if(tAbs < huberK_ * sig)
    return 1.0;
  else
    return sig*huberK_ / tAbs;
}

void Tracker::CheckVisiblePointsInPrevFrame(Frame::Ptr currFrame, Sophus::SE3f& transformation){
  const int border = patch_halfsize_+2;
  cv::Mat& current_img = currFrame->GetOriginalImg();

  const float scale = 1.0f;
  std::vector<bool>::iterator visibilityIterPrev = visiblePointsPrev_.begin();

  for (auto iter=currFrame->GetOriginalCloud().begin(); iter!=currFrame->GetOriginalCloud().end(); ++iter, ++visibilityIterPrev) {
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

    *visibilityIterPrev = true;
  }
}

void Tracker::PrecomputeReferencePatterns(){
  const int border = patch_halfsize_ + 2;
  cv::Mat& referenceImg = referenceFrame_->frame->GetOriginalImg();
  const int stride = referenceImg.cols;
  const float scale = 1.0f;

  size_t pointCounter = 0;
  std::vector<bool>::iterator visibilityIter = visiblePoints_.begin();

  cv::Mat zBuff;
  zBuff = cv::Mat::zeros(referenceImg.size(), CV_8U);

  for(auto iter=referenceFrame_->frame->GetOriginalCloud().begin(); iter!=referenceFrame_->frame->GetOriginalCloud().end(); ++iter, ++pointCounter, ++visibilityIter){
    Eigen::Vector3f xyzRef (iter->x, iter->y, iter->z);
    Eigen::Vector2f uvRef;
    uvRef.noalias() = referenceFrame_->frame->PointCloudXyz2Uv(xyzRef) * scale;

    const float uRefFloat = uvRef(0);
    const float vRefFloat = uvRef(1);
    const int uRefInt = static_cast<int> (uRefFloat);
    const int vRefInt = static_cast<int> (vRefFloat);

    if (zBuff.at<uint8_t>(uRefInt, uRefInt) == 1) continue;
    zBuff.at<uint8_t>(uRefInt, uRefInt) = 1;

    *visibilityIter = true;

    Matrix2x6 frameJac;
    Frame::jacobian_xyz2uv(xyzRef, frameJac);

    const float subpixURef = uRefFloat - uRefInt;
    const float subpixVRef = vRefFloat - vRefInt;
    const float wRefTL = (1.0-subpixURef) * (1.0-subpixURef);
    const float wRefTR = subpixURef * (1.0-subpixVRef);
    const float wRefBL = (1.0-subpixURef) * subpixVRef;
    const float wRefBR = subpixURef * subpixVRef;

    size_t pixelCounter = 0;
    float* refPatchBufPtr = reinterpret_cast<float *> (refPatchBuf_.data) + patch_area_ * pointCounter;

    for(int y=0; y<patch_size_; ++y){
      float *referenceImgPtr = (float*) referenceImg.data + (vRefInt + y - patch_halfsize_) * stride + (uRefInt - patch_halfsize_);
      for(int x=0; x<patch_size_; ++x, ++referenceImgPtr, ++refPatchBufPtr, ++pixelCounter){
        *refPatchBufPtr = wRefTL * referenceImgPtr[0] + wRefTR * referenceImgPtr[1] + wRefBL * referenceImgPtr[stride+1] + wRefBR * referenceImgPtr[stride+2];
        float dx = 0.5f * ((wRefTL * referenceImgPtr[1] + wRefTR * referenceImgPtr[2] + wRefBL * referenceImgPtr[stride+1] + wRefBR * referenceImgPtr[stride+2])
                    - (wRefTL * referenceImgPtr[-1] + wRefTR * referenceImgPtr[0] + wRefBL * referenceImgPtr[stride-1] + wRefBR * referenceImgPtr[stride]));
        float dy = 0.5f * ((wRefTL * referenceImgPtr[stride] + wRefTR * referenceImgPtr[1+stride] + wRefBL * referenceImgPtr[stride*2] + wRefBR * referenceImgPtr[stride*2 + 1])
                    -(wRefTL * referenceImgPtr[-stride] + wRefTR * referenceImgPtr[1-stride] + wRefBL * referenceImgPtr[0] + wRefBR * referenceImgPtr[1]));

        jacobianBuf_.col(pointCounter*patch_area_ + pixelCounter) = (dx*config_.fx * frameJac.row(0) + dy*config_.fy*frameJac.row(1)) /  (1);
      }
    }
  }
  isSetRefPatch_ = true;
}

double Tracker::ComputeResidualsPatterns(Sophus::SE3f &transformation){
  errors_.clear();
  cv::Mat& currImg = currentFrame_->GetOriginalImg();

  if(!isSetRefPatch_){
    PrecomputeReferencePatterns();
  }


  errors_.reserve(visiblePoints_.size());

  const int stride = currImg.cols;
  const int border = patch_halfsize_+2;
  const float scale = 1.0f/(1<<currentLevel_);

  float chi2 = 0.0;
  size_t pointCounter = 0;
  std::vector<bool>::iterator visibilityIter = visiblePoints_.begin();
  std::vector<bool>::iterator visibilityIterCur = visiblePointsInCur_.begin();

  for (auto iter = referenceFrame_->frame->GetOriginalCloud().begin(); iter!=referenceFrame_->frame->GetOriginalCloud().end(); ++iter, ++pointCounter, ++visibilityIterCur){
    Eigen::Vector3f xyzRef(iter->x, iter->y, iter->z);
    Eigen::Vector3f xyzCur = transformation * xyzRef;
    Eigen::Vector2f uvCur;
    uvCur.noalias() = referenceFrame_->frame->PointCloudXyz2Uv(xyzCur) * scale;

    const float uCurFloat = uvCur(0);
    const float vCurFloat = uvCur(1);
    const int uCurInt = static_cast<int>(uCurFloat);
    const int vCurInt = static_cast<int>(vCurFloat);

    if(uCurInt - border < 0 || uCurInt + border > currImg.cols || vCurInt - border < 0 || vCurInt + border > currImg.cols)
      continue;

    *visibilityIterCur = true;

    const float subpixUCur = uCurFloat - uCurInt;
    const float subpixVCur = vCurFloat - vCurInt;
    const float wCurTL = (1.0-subpixUCur) * (1.0-subpixUCur);
    const float wCurTR = subpixUCur * (1.0-subpixVCur);
    const float wCurBL = (1.0-subpixUCur) * subpixVCur;
    const float wCurBR = subpixUCur * subpixVCur;

    float* refPatchBufPtr = reinterpret_cast<float *> (refPatchBuf_.data) + pattern_length_ * pointCounter;

    size_t pixelCounter = 0;

    for(int i=0; i<pattern_length_; ++i, ++pixelCounter, ++refPatchBufPtr){
      int x = pattern_[i][0];
      int y = pattern_[i][1];

      float* currImgPtr = (float*)currImg.data + (vCurInt + y) * stride + (uCurInt + x);
      float intensityCur = wCurTL*currImgPtr[0] + wCurTR*currImgPtr[1] + wCurBL*currImgPtr[stride] + wCurBR*currImgPtr[stride+1];
      float res = intensityCur - (*refPatchBufPtr);

      float weight = 1.0;

      weight = HuberWeight(res/scale);
      errors_.push_back(res);

      chi2+=res*res*weight;

      nMeasurement_++;

      Vector6 J(jacobianBuf_.col(pointCounter*pattern_length_ + pixelCounter));
      H_.noalias() += J*J.transpose()*weight;
      Jres_.noalias() -= J*res*weight;
    }
  }
  return chi2/nMeasurement_;

}

Sophus::SE3f Tracker::trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr keyFrame)
{

}


