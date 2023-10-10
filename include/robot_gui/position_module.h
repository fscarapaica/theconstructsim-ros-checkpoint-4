#ifndef _POSITION_MODULE_H_
#define _POSITION_MODULE_H_
#include "robot_info_gui.h"
#include "ros/subscriber.h"
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

class UIPositionModule : public UIModule {
  ros::Subscriber robot_position_subscriber;
  float positions[3] = {0};

public:
  UIPositionModule(ros::NodeHandle *nh);
  ~UIPositionModule();
  int render(cv::Mat &frame) override;
  void robotPositionCallback(const nav_msgs::Odometry::ConstPtr &odom_info);
};

#endif