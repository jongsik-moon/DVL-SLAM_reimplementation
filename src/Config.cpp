//
// Created by jongsik on 20. 10. 28..
//

#include "Config.h"
#include <iostream>

Config::Config()
{
  YAML::Node yamlFile = YAML::LoadFile("/home/jongsik/modu_ws/src/dvl_slam_modify/yaml/defalut_param.yaml");
  ReadEveryParameter(yamlFile);
}


Config::~Config()
{


}


void Config::ReadEveryParameter(const YAML::Node yamlFile)
{
  YAML::Node cameraYaml = yamlFile["Camera"];
  YAML::Node extrinsicYaml = yamlFile["Extrinsic"];

  fx = cameraYaml["fx"].as<float>();
  fy = cameraYaml["fy"].as<float>();
  cx = cameraYaml["cx"].as<float>();
  cy = cameraYaml["cy"].as<float>();
  k1 = cameraYaml["k1"].as<float>();
  k2 = cameraYaml["k2"].as<float>();
  p1 = cameraYaml["p1"].as<float>();
  p2 = cameraYaml["p2"].as<float>();
  k3 = cameraYaml["k3"].as<float>();

  delX = extrinsicYaml["delX"].as<float>();
  delY = extrinsicYaml["delY"].as<float>();
  delZ = extrinsicYaml["delZ"].as<float>();
}