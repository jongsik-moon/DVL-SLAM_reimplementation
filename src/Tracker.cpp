//
// Created by jongsik on 20. 10. 30..
//
#include "Tracker.h"

Tracker::Tracker(Config &config)
  : config_(config)
  , pinholeModel_(config)
{
  huberK_ = 1.345;
  residual_ = 0;

  normXThres_ = config_.trackerConfig.normXThresForIteration;
  minLevel_ = config_.trackerConfig.imgPyramidMinLevel;
  maxLevel_ = config_.trackerConfig.imgPyramidMaxLevel;
  maxIteration = config_.trackerConfig.maxIteration;

}

Tracker::~Tracker(){

}

bool Tracker::Solve(){
  x_ = H_.ldlt().solve(Jres_);
//  std::cout << "[Optimize] Jres_ = " << Jres_ << std::endl;
//  std::cout << "[Optimize] H_ = " << H_ << std::endl;
//  std::cout << "[Optimize] x_ = " << x_ << std::endl;


  if ( ((x_ - x_).array() == (x_ - x_).array()).all() )
    return true;
  return false;
}

void Tracker::UpdatePose(const Sophus::SE3f& old_Tji, Sophus::SE3f& Tji){
  Tji = old_Tji * Sophus::SE3f::exp(-x_);
//  std::cout << "[Tracker] old_Tji = " << old_Tji.matrix() << std::endl;
//  std::cout << "[Tracker] Sophus::SE3f::exp(-x_) = " << Sophus::SE3f::exp(-x_).matrix() << std::endl;
//  std::cout << "[Tracker] Tji = " << Tji.matrix() << std::endl;
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
    std::cout << "[Optimize] newResidual:" << newResidual << std::endl;

    for(int j=0; j < errors_.size(); ++j) {
      float& res = errors_[j];
      Vector6& J = J_[j];
//      float& weight = weight_[j];
      float weight = 1;

      H_.noalias() += J*J.transpose()*weight;
      Jres_.noalias() -= J*res*weight;
//      std::cout << "[Optimize] J : " << J << std::endl;

    }

    if(!Solve()){
      stop_ = true;
      std::cout << "[Optimize] Stop Sign From Solve" << std::endl;

    }
//    if(i > 0 && newResidual > residual_ || stop_){
//      Tji = oldTji;
//      H_ = oldH;
//      std::cout << "[Optimize] Stop Sign From Res" << std::endl;
//      break;
//    }
    Sophus::SE3f newTji;
    UpdatePose(Tji, newTji);
    oldTji = Tji;
    oldH = H_;
    Tji = newTji;

    residual_ = newResidual;
    std::cout << "[Optimize] NormMax(x_) = " << NormMax(x_) << std::endl;

    if(NormMax(x_) < normXThres_){
      status_ = true;

      if ( ((x_ - x_).array() != (x_ - x_).array()).all() )
        status_ = false;
      std::cout << "[Optimize] break from normmax" << std::endl;

      break;
    }
  }
  std::cout << "[Optimize] Iteration Finished" << std::endl;

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

cv::Mat Tracker::PrecomputePatches(cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, cv::Mat& patchBuf, bool isDerivative){
  int border = config_.trackerConfig.border;
  int stride = img.cols;
  float scale = 1.0f/(1<<currentLevel_);

  std::vector<Eigen::Vector2f> uvSet = pinholeModel_.PointCloudXyz2UvVec(pointcloud, scale);
  patchBuf = cv::Mat(pointcloud.size(), patternLength_, CV_32F);
  if(isDerivative){
    jacobianBuf_.resize(Eigen::NoChange, patchBuf.rows * patternLength_);
    jacobianBuf_.setZero();
  }

  auto pointCloudIter = pointcloud.begin();
  size_t pointCounter = 0;

  int pointNum = pointcloud.points.size();
  cv::Mat imgClone = cv::Mat(img.rows, img.cols, CV_32FC3, cv::Scalar(0));

//  cv::cvtColor(imgClone, imgClone, cv::COLOR_GRAY2BGR);
//  for(int i=0; i<img.rows; i++){
//    for(int j=0; j<img.cols; j++){
//      std::cout << "[Optimize] imgClone.at<float>(uInt, vInt)0 = " << imgClone.at<float>(i, j) << std::endl;
//      std::cout << "[Optimize] img.at<float>(uInt, vInt)0 = " << img.at<float>(i, j) << std::endl;
//    }
//  }

  float temp;
  for(auto uvIter=uvSet.begin(); uvIter!=uvSet.end(); ++uvIter, ++pointCloudIter, ++pointCounter){
    temp = 0;
    Eigen::Vector2f uv = *uvIter;

    const float uFloat = uv(0);
    const float vFloat = uv(1);
    const int uInt = static_cast<int> (uFloat);
    const int vInt = static_cast<int> (vFloat);

    if(uInt - border < 0 || uInt + border > img.cols || vInt - border < 0 || vInt + border > img.rows || pointCloudIter->z <= 0.0){
      float* patchBufPtr = reinterpret_cast<float *> (patchBuf.data) + patternLength_ * pointCounter;
      for(int i=0; i<patternLength_; ++i, ++patchBufPtr)
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
    float* patchBufPtr = reinterpret_cast<float *> (patchBuf.data) + patternLength_ * pointCounter;

    for (int i=0; i<patternLength_; ++i, ++pixelCounter, ++patchBufPtr){
      int x = pattern_[i][0];
      int y = pattern_[i][1];

      float* imgPtr = (float*) img.data + (vInt + y) * stride + (uInt + x);
      *patchBufPtr = wTL * imgPtr[0] + wTR * imgPtr[1] + wBL * imgPtr[stride] + wBR * imgPtr[stride + 1];
      temp += *patchBufPtr;
     if(isDerivative){
       float dx = 0.5f * ((wTL*imgPtr[1] + wTR*imgPtr[2] + wBL*imgPtr[stride+1] + wBR*imgPtr[stride+2])
                          -(wTL*imgPtr[-1] + wTR*imgPtr[0] + wBL*imgPtr[stride-1] + wBR*imgPtr[stride]));
       float dy = 0.5f * ((wTL*imgPtr[stride] + wTR*imgPtr[1+stride] + wBL*imgPtr[stride*2] + wBR*imgPtr[stride*2+1])
                          -(wTL*imgPtr[-stride] + wTR*imgPtr[1-stride] + wBL*imgPtr[0] + wBR*imgPtr[1]));

       Matrix2x6 frameJacobian;
       Eigen::Vector3f xyz(pointCloudIter->x, pointCloudIter->y, pointCloudIter->z);
       Frame::jacobian_xyz2uv(xyz, frameJacobian);

       jacobianBuf_.col(pointCounter*patternLength_ + pixelCounter) = (dx*config_.cameraConfig.fx * frameJacobian.row(0) + dy*config_.cameraConfig.fy*frameJacobian.row(1)) / (1 << currentLevel_);
     }
    }
    temp /= patternLength_;
    cv::Scalar_<float> color = cv::Scalar_<float>(temp, temp, temp);
    cv::circle(imgClone, cv::Point(uInt, vInt), 2, color, 1);
  }
//  std::cout << "[Tracker] patchBuf.rows = " << patchBuf.rows << std::endl;
//  std::cout << "[Tracker] pointcloud.size() = " << pointcloud.size() << std::endl;
//  std::cout << "[Tracker] jacobianBuf_.size() = " << jacobianBuf_.size() << std::endl;

  return imgClone;
}

double Tracker::ComputeResiduals(Sophus::SE3f &transformation)
{
  errors_.clear();
  J_.clear();
  weight_.clear();

  if (!isPreComputed_)
  {
    cv::Mat &referenceImg = referenceFrame_->frame->GetPyramidImg(currentLevel_);
    pcl::PointCloud<pcl::PointXYZRGB> &referencePointCloud = referenceFrame_->frame->GetOriginalCloud();
    refImgClone = PrecomputePatches(referenceImg, referencePointCloud, refPatchBuf_, true);
    isPreComputed_ = true;
  }
  cv::Mat &currImg = currentFrame_->GetPyramidImg(currentLevel_);
  pcl::PointCloud<pcl::PointXYZRGB> currPointCloud;
  pcl::transformPointCloud(referenceFrame_->frame->GetOriginalCloud(), currPointCloud, transformation.matrix());
  currImgClone = PrecomputePatches(currImg, currPointCloud, currPatchBuf_, false);
  cv::Mat errors = cv::Mat(currPointCloud.size(), patternLength_, CV_32F);
  errors = currPatchBuf_ - refPatchBuf_;

  if(config_.datasetConfig.visualize){
    cv::Mat diff = currImgClone - refImgClone;
    cv::Mat temp1, temp2, temp3;
    temp1 = refImgClone.clone();
    temp2 = currImgClone.clone();
    temp3 = diff.clone();
    temp1.convertTo(temp1, CV_8U, 255);
    temp2.convertTo(temp2, CV_8U, 255);
    temp3.convertTo(temp3, CV_8U, 255);
    cv::imshow("refImgClone", temp1);
    cv::imshow("currImgClone", temp2);
    cv::imshow("diff", diff);
    cv::moveWindow("refImgClone", 40, 30);
    cv::moveWindow("currImgClone", 40, 500);
    cv::moveWindow("diff", 40, 900);

    while(1){
      int key = cv::waitKey(0);
      if(key == 27) break;
    }
  }

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

//    std::cout << "[Tracker] i = " << i << std::endl;
//    std::cout << "[Tracker] res = " << res << std::endl;
//    std::cout << "[Tracker] Ii = " << Ii << std::endl;
//    std::cout << "[Tracker] Ij = " << Ij << std::endl;

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
//    weight = calcWeight((error - medianMu) / medianAbsDeviation);
    weight_.push_back(weight);
    chi2 += error * error * weight;
//    std::cout << "[Tracker] weight = " << weight << std::endl;
//    std::cout << "[Tracker] error = " << error << std::endl;
  }

  return chi2 / nMeasurement;
}

void Tracker::trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr refFrame, Sophus::SE3f& transformation)
{
  currentFrame_ = currFrame;
  referenceFrame_ = refFrame;

  affine_a_ = 1.0f;
  affine_b_ = 0.0f;

  std::cout << "[Tracker] Try to Optimize" << std::endl;

  for(currentLevel_ = maxLevel_; currentLevel_ >= minLevel_; currentLevel_--){
    isPreComputed_ = false;
    stop_ = false;
    Optimize(transformation);
  }
  std::cout << "[Tracker] Optimization finished" << std::endl;

}
