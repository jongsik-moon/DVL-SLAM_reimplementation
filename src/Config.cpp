//
// Created by jongsik on 20. 10. 28..
//

#include "Config.h"
#include <iostream>

Config::Config()
{
  YAML::Node yamlFile = YAML::LoadFile("../yaml/default_param.yml");
  ReadEveryParameter(yamlFile);
}


Config::~Config()
{


}


void Config::ReadEveryParameter(const YAML::Node yamlFile)
{
  fx = yamlFile["Camera"][0]["fx"].as<float>();
  fy = yamlFile["Camera"][1]["fy"].as<float>();
  cx = yamlFile["Camera"][2]["cx"].as<float>();
  cy = yamlFile["Camera"][3]["cy"].as<float>();

  k1 = yamlFile["Camera"][4]["k1"].as<float>();
  k2 = yamlFile["Camera"][5]["k2"].as<float>();
  k3 = yamlFile["Camera"][6]["k3"].as<float>();
  p1 = yamlFile["Camera"][7]["p1"].as<float>();
  p2 = yamlFile["Camera"][8]["p2"].as<float>();

  delX = yamlFile["Extrinsic"][0]["delX"].as<float>();
  delY = yamlFile["Extrinsic"][1]["delY"].as<float>();
  delZ = yamlFile["Extrinsic"][2]["delZ"].as<float>();

}