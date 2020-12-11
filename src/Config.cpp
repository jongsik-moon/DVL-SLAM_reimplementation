//
// Created by jongsik on 20. 10. 28..
//

#include "Config.h"
#include <iostream>

Config::Config()
{
  YAML::Node yamlFile = YAML::LoadFile("/home/jongsik/modu_ws/src/dvl_slam_modify/yaml/defalut_param_kitti.yaml");
  ReadEveryParameter(yamlFile);
}


Config::~Config()
{


}


void Config::ReadEveryParameter(const YAML::Node yamlFile)
{
  YAML::Node datasetYaml = yamlFile["Dataset"];
  YAML::Node cameraYaml = yamlFile["Camera"];
  YAML::Node extrinsicYaml = yamlFile["Extrinsic"];
  YAML::Node rectifyingYaml = yamlFile["Rectifying"];

  isKitti = datasetYaml["isKitti"].as<bool>();
  isIndoor = datasetYaml["isIndoor"].as<bool>();
  useRos = datasetYaml["useRos"].as<bool>();

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

  r11 = extrinsicYaml["r11"].as<float>();
  r12 = extrinsicYaml["r12"].as<float>();
  r13 = extrinsicYaml["r13"].as<float>();
  r21 = extrinsicYaml["r21"].as<float>();
  r22 = extrinsicYaml["r22"].as<float>();
  r23 = extrinsicYaml["r23"].as<float>();
  r31 = extrinsicYaml["r31"].as<float>();
  r32 = extrinsicYaml["r32"].as<float>();
  r33 = extrinsicYaml["r33"].as<float>();

  R11 = rectifyingYaml["R11"].as<float>();
  R12 = rectifyingYaml["R12"].as<float>();
  R13 = rectifyingYaml["R13"].as<float>();
  R21 = rectifyingYaml["R21"].as<float>();
  R22 = rectifyingYaml["R22"].as<float>();
  R23 = rectifyingYaml["R23"].as<float>();
  R31 = rectifyingYaml["R31"].as<float>();
  R32 = rectifyingYaml["R32"].as<float>();
  R33 = rectifyingYaml["R33"].as<float>();

}