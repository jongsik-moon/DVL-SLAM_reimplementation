#include <ros/ros.h>
#include <iostream>
#include <string>
#include "Config.h"
#include "System.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "dvl_reimpl");
  ros::NodeHandle nh;

  ros::Rate rate(20.0f);

  Config Config;
  System System(Config);

  while(ros::ok()){
    System.Run();
    ros::spinOnce();
  }
}