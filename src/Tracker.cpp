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

  pinholeModel_ = std::static_pointer_cast<PinholeModel> (pinholeModel_);
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
    std::cout << "[Optimize] Iteration : " << i << std::endl;

    H_.setZero();
    Jres_.setZero();

    double newResidual = ComputeResiduals(Tji);

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

    residual_ = newResidual;
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

//void Tracker::CheckVisiblePointsInPrevFrame(Frame::Ptr currFrame, Sophus::SE3f& transformation){
//  const int border = patch_halfsize_+2;
//  cv::Mat& current_img = currFrame->GetOriginalImg();
//
//  const float scale = 1.0f;
//  std::vector<bool>::iterator visibilityIterPrev = visiblePointsPrev_.begin();
//
//  for (auto iter=currFrame->GetOriginalCloud().begin(); iter!=currFrame->GetOriginalCloud().end(); ++iter, ++visibilityIterPrev) {
//    Eigen::Vector3f xyz_cur (iter->x, iter->y, iter->z);
//    Eigen::Vector3f xyz_prev = transformation.inverse()*xyz_cur;
//    Eigen::Vector2f uv_prev;
//    uv_prev.noalias() = currFrame->PointCloudXyz2Uv(xyz_prev) * scale;
//
//    const float u_prev_f = uv_prev(0);
//    const float v_prev_f = uv_prev(1);
//    const int u_prev_i = static_cast<int> (u_prev_f);
//    const int v_prev_i = static_cast<int> (v_prev_f);
//
//    if (u_prev_i - border < 0 || u_prev_i + border > current_img.cols || v_prev_i - border < 0 || v_prev_i + border > current_img.rows || xyz_prev(2) <= 0)
//      continue;
//
//    *visibilityIterPrev = true;
//  }
//}

void Tracker::PrecomputePatches(cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, cv::Mat& patchBuf, bool isDerivative){
  const int border = patch_halfsize_ + 4;
  const int stride = img.cols;
  const float scale = 1.0f;

  std::vector<Eigen::Vector2f> uvSet = pinholeModel_->PointCloudXyz2UvVec(pointcloud);
  patchBuf = cv::Mat(pointcloud.size(), pattern_length_, CV_32F);

  if(isDerivative){
    dIBuf_.resize(Eigen::NoChange, patchBuf.rows * pattern_length_);
    jacobianBuf_.resize(Eigen::NoChange, patchBuf.rows * pattern_length_);
    jacobianBuf_.setZero();
  }

  auto pointCloudIter = pointcloud.begin();
  size_t pointCounter = 0;

  for(auto uvIter=uvSet.begin(); uvIter!=uvSet.end(); ++uvIter, ++pointCloudIter, ++pointCounter){
    Eigen::Vector2f uv = *uvIter;

    const float uFloat = uv(0);
    const float vFloat = uv(1);
    const int uInt = static_cast<int> (uFloat);
    const int vInt = static_cast<int> (vFloat);

    if(uInt - border < 0 || uInt + border > img.cols || vInt - border < 0 || vInt + border > img.cols){
      float* patchBufPtr = reinterpret_cast<float *> (patchBuf.data) + pattern_length_ * pointCounter;
      for(int i=0; i<pattern_length_; ++i, ++patchBufPtr)
        *patchBufPtr = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    const float subpixURef = uFloat - uInt;
    const float subpixVRef = vFloat - vInt;
    const float wTL = (1.0-subpixURef) * (1.0-subpixURef);
    const float wTR = subpixURef * (1.0-subpixVRef);
    const float wBL = (1.0-subpixURef) * subpixVRef;
    const float wBR = subpixURef * subpixVRef;

    size_t pixelCounter = 0;
    float* patchBufPtr = reinterpret_cast<float *> (refPatchBuf_.data) + patch_area_ * pointCounter;

    for (int i=0; i<pattern_length_; ++i, ++pixelCounter, ++patchBufPtr){
     int x = pattern_[i][0];
     int y = pattern_[i][1];

     float* imgPtr = (float*) img.data + (vInt + y) * stride + (uInt + x);
     *patchBufPtr = wTL * imgPtr[0] + wTR * imgPtr[1] + wBL * imgPtr[stride] + wBR * imgPtr[stride + 1];

     if(isDerivative){
       float dx = 0.5f * ((wTL*imgPtr[1] + wTR*imgPtr[2] + wBL*imgPtr[stride+1] + wBR*imgPtr[stride+2])
                          -(wTL*imgPtr[-1] + wTR*imgPtr[0] + wBL*imgPtr[stride-1] + wBR*imgPtr[stride]));
       float dy = 0.5f * ((wTL*imgPtr[stride] + wTR*imgPtr[1+stride] + wBL*imgPtr[stride*2] + wBR*imgPtr[stride*2+1])
                          -(wTL*imgPtr[-stride] + wTR*imgPtr[1-stride] + wBL*imgPtr[0] + wBR*imgPtr[1]));
       Matrix2x6 frameJacobian;
       Eigen::Vector3f xyz(pointCloudIter->x, pointCloudIter->y, pointCloudIter->z);
       Frame::jacobian_xyz2uv(xyz, frameJacobian);

       Eigen::Vector2f dIxy(dx, dy);
       dIBuf_.col(pointCounter*pattern_length_ + i) = dIxy;
       jacobianBuf_.col(pointCounter*pattern_length_ + pixelCounter) = (dx*config_.fx * frameJacobian.row(0) + dy*config_.fy*frameJacobian.row(1));
     }
    }
  }
}

double Tracker::ComputeResiduals(Sophus::SE3f &transformation)
{
  errors_.clear();
  J_.clear();
  weight_.clear();

  if (!isPreComputed_)
  {
    cv::Mat &referenceImg = referenceFrame_->frame->GetOriginalImg();
    pcl::PointCloud<pcl::PointXYZRGB> &referencePointCloud = referenceFrame_->frame->GetOriginalCloud();
    PrecomputePatches(referenceImg, referencePointCloud, refPatchBuf_, true);
    isPreComputed_ = true;
  }
  cv::Mat &currImg = currentFrame_->GetOriginalImg();
  pcl::PointCloud<pcl::PointXYZRGB> currPointCloud;
  pcl::transformPointCloud(referenceFrame_->frame->GetOriginalCloud(), currPointCloud, transformation.matrix());

  PrecomputePatches(currImg, currPointCloud, currPatchBuf_, false);

  cv::Mat errors = cv::Mat(currPointCloud.size(), pattern_length_, CV_32F);
  errors = currPatchBuf_ - refPatchBuf_;

  scale_ = compute(errors);

  float chi2 = 0.0;
  size_t nMeasurement = 0;

  float *errorsPtr = errors.ptr<float>();
  float *refPatchBufPtr = refPatchBuf_.ptr<float>();
  float *currPatchBufPtr = currPatchBuf_.ptr<float>();

  float IiIj = 0.0f;
  float IiIi = 0.0f;
  float sumIi = 0.0f;
  float sumIj = 0.0f;

  for (int i = 0; i < errors.size().area(); ++i, ++errorsPtr, ++refPatchBufPtr, ++currPatchBufPtr)
  {
    float &res = *errorsPtr;
    float &Ii = *refPatchBufPtr;
    float &Ij = *currPatchBufPtr;
    if (std::isfinite(res))
    {
      nMeasurement++;
      Vector6 J(jacobianBuf_.col(i));
      errors_.push_back(res);
      J_.push_back(J);
      IiIj += Ii * Ij;
      IiIi += Ii * Ii;
      sumIi += Ii;
      sumIj += Ij;
    }
    affine_a_ = IiIj / IiIi;
    affine_b_ = (sumIj - affine_a_ * sumIi) / nMeasurement;

    std::vector<float> sortedErrors;
    sortedErrors.resize(errors_.size());
    std::copy(errors_.begin(), errors_.end(), sortedErrors.begin());
    std::sort(sortedErrors.begin(), sortedErrors.end());

    float medianMu = sortedErrors[sortedErrors.size() / 2];
    std::vector<float> absoluteResError;
    for (auto error:errors_)
    {
      absoluteResError.push_back(fabs(error - medianMu));
    }
    sort(absoluteResError.begin(), absoluteResError.end());
    float medianAbsDeviation = 1.4826 * absoluteResError[absoluteResError.size() / 2];

    for (auto error:errors_)
    {
      float weight = 1.0;
      weight = calcWeight((error - medianMu) / medianAbsDeviation);
      weight_.push_back(weight);
      chi2 += error * error * weight;
    }
    return chi2 / nMeasurement;
  }
}

bool Tracker::trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr refFrame, Sophus::SE3f& transformation)
{
  currentFrame_ = currFrame;
  referenceFrame_ = refFrame;

  affine_a_ = 1.0f;
  affine_b_ = 0.0f;

  isPreComputed_ = false;
  stop_ = false;

  std::cout << "[Tracker] Try to Optimize" << std::endl;

  Optimize(transformation);

  return true;

}