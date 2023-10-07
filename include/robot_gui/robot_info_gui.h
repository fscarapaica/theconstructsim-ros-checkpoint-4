#pragma once
#ifndef ROBOT_INFO_GUI_H_
#define ROBOT_INFO_GUI_H_

#include "cvui.h"

#include "ros/node_handle.h"
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <ros/ros.h>
#include <utility>
#include <vector>

const static int DEFAULT_WIDTH_SIZE = 300;

class UIModule {
public:
  int max_width = DEFAULT_WIDTH_SIZE;
  int start_render_y_at = 0;

  virtual int render(cv::Mat &frame) = 0;
};

class RobotInfoGui {
public:
  // Constructor
  RobotInfoGui(){};

  // Destructor
  ~RobotInfoGui();

  void addModule(UIModule *module);

  void show();

private:
  std::vector<UIModule *> ui_modules;
};

#endif
