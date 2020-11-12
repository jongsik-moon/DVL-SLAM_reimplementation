//
// Created by jongsik on 20. 10. 28..
//

#ifndef DVL_SLAM_MODIFY_CONFIG_H
#define DVL_SLAM_MODIFY_CONFIG_H

#include <yaml-cpp/yaml.h>

struct Camera
{
  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float p1;
  float p2;
  float k3;
};

struct Extrinsic
{
  float delX;
  float delY;
  float delZ;
};

class Config{
public:
  Config();
  ~Config();

  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float k3;
  float p1;
  float p2;

  float delX;
  float delY;
  float delZ;

  Camera camera;
  Extrinsic extrinsic;

private:
  void ReadEveryParameter(YAML::Node yamlFile);
};

#endif //DVL_SLAM_MODIFY_CONFIG_H
