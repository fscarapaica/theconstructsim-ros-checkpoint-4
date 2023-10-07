#ifndef _DISTANCE_MODULE_H_
#define _DISTANCE_MODULE_H_

#include "robot_info_gui.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class UIDistanceModule : public UIModule {
private: 
  ros::ServiceClient distance_service;
  std_srvs::Trigger trigger;
  std::string last_distance = "";

public:
  UIDistanceModule(ros::NodeHandle *nh);
  ~UIDistanceModule();
  int render(cv::Mat &frame) override;
};

#endif