#ifndef _INFO_PANEL_MODULE_H_
#define _INFO_PANEL_MODULE_H_
#include "robot_info_gui.h"
#include "ros/subscriber.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>

class UIInfoPanelModule : public UIModule {
private:
  ros::Subscriber robot_info_subscriber;
  std::string msg_fields[10];

public:
  UIInfoPanelModule(ros::NodeHandle *nh);
  ~UIInfoPanelModule();
  int render(cv::Mat &frame) override;
  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info);
};

#endif
