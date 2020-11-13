//
// Created by jongsik on 20. 10. 30..
//

#include "System.h"

System::System(const Config& config)
  : config_(config),
    frame_(config_),
    graphOptimizer_(config_),
    keyFrame_(config_),
    sensor_(config_)
{


}

System::~System(){


}


void System::Run(){

  std::cout << "system run" << std::endl;
  sensor_.data2Frame(frame_);
  sensor_.publishImg(frame_.pointCloudProjection());


}