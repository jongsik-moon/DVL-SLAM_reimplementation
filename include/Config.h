//
// Created by jongsik on 20. 10. 28..
//

#ifndef DVL_SLAM_MODIFY_CONFIG_H
#define DVL_SLAM_MODIFY_CONFIG_H

#include <yaml-cpp/yaml.h>
struct Dataset
{
  bool isKitti;
  bool isIndoor;
};

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

struct Rectifying
{
  float R11;
  float R12;
  float R13;
  float R21;
  float R22;
  float R23;
  float R31;
  float R32;
  float R33;
};

class Config{
public:
  Config();
  ~Config();

  bool isKitti;
  bool isIndoor;

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

  float R11;
  float R12;
  float R13;
  float R21;
  float R22;
  float R23;
  float R31;
  float R32;
  float R33;

  Dataset dataset;
  Camera camera;
  Extrinsic extrinsic;
  Rectifying rectifying;

private:
  void ReadEveryParameter(YAML::Node yamlFile);
};

#endif //DVL_SLAM_MODIFY_CONFIG_H
