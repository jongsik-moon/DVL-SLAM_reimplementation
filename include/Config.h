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

  float r11;
  float r12;
  float r13;
  float r21;
  float r22;
  float r23;
  float r31;
  float r32;
  float r33;
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

  float r11;
  float r12;
  float r13;
  float r21;
  float r22;
  float r23;
  float r31;
  float r32;
  float r33;

  Camera camera;
  Extrinsic extrinsic;

private:
  void ReadEveryParameter(YAML::Node yamlFile);
};

#endif //DVL_SLAM_MODIFY_CONFIG_H
