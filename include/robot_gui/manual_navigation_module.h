#ifndef _MANUAL_NAVIGATION_MODULE_H_
#define _MANUAL_NAVIGATION_MODULE_H_
#include "robot_info_gui.h"
#include "ros/publisher.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class UIManualNavigationModule : public UIModule {
private:  
  ros::Publisher twist_publisher;
  geometry_msgs::Twist twist_msg;
  float velocity_step = 0.1;

public:
  UIManualNavigationModule(ros::NodeHandle *nh);
  ~UIManualNavigationModule();
  int render(cv::Mat &frame) override;
};

#endif
