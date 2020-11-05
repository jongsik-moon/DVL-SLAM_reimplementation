//
// Created by jongsik on 20. 10. 30..
//

#include "System.h"

System::System(const Config& config)
  : config_(config),
    frame_(new Frame(config_)),
    graphOptimizer_(new GraphOptimizer(config_)),
    keyFrame_(new KeyFrame(config_)),
    sensor_(new Sensor(config_)),

{


}

System::~System(){


}


void System::Run(){



}