#include "robot_gui/position_module.h"
#include "ros/init.h"
#include <robot_gui/distance_module.h>
#include <robot_gui/info_panel_module.h>
#include <robot_gui/manual_navigation_module.h>
#include <robot_gui/robot_info_gui.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_info_node");
  ros::NodeHandle nh;
  RobotInfoGui robot_gui;
  UIManualNavigationModule navigation_module(&nh);
  UIInfoPanelModule info_panel_module(&nh);
  UIPositionModule position_module(&nh);
  UIDistanceModule distance_module(&nh);
  robot_gui.addModule(&info_panel_module);
  robot_gui.addModule(&navigation_module);
  robot_gui.addModule(&position_module);
  robot_gui.addModule(&distance_module);
  robot_gui.show();
  ros::spin();
}
