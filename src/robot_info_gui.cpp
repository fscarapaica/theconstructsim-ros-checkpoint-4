// External dependencies
#include "ros/node_handle.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <std_srvs/Trigger.h>

// Project Local dependencies
#include <robot_gui/distance_module.h>
#include <robot_gui/info_panel_module.h>
#include <robot_gui/manual_navigation_module.h>
#include <robot_gui/position_module.h>
#include <robot_gui/robot_info_gui.h>

#define CVUI_IMPLEMENTATION
#include <robot_gui/cvui.h>

#define WINDOW_NAME "Robot Info Gui"

/* ROBOT INFO WINDOW */
void RobotInfoGui::addModule(UIModule *module) { ui_modules.push_back(module); }

void RobotInfoGui::show() {
  cvui::init(WINDOW_NAME);
  cv::Mat frame = cv::Mat(cv::Size(270, 550), CV_8UC3);

  int render_at;
  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    render_at = 0;
    for (auto &element : ui_modules) {
      element->start_render_y_at = render_at;
      render_at = element->render(frame);
    }

    // Update cvui internal stuff
    cvui::update();

    // Show window content
    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}

RobotInfoGui::~RobotInfoGui() { ui_modules.clear(); }
/* END WINDOW */

/* INFO MODULE */
void UIInfoPanelModule::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &robot_info) {

  msg_fields[0] = robot_info->data_field_01;
  msg_fields[1] = robot_info->data_field_02;
  msg_fields[2] = robot_info->data_field_03;
  msg_fields[3] = robot_info->data_field_04;
  msg_fields[4] = robot_info->data_field_05;
  msg_fields[5] = robot_info->data_field_06;
  msg_fields[6] = robot_info->data_field_07;
  msg_fields[7] = robot_info->data_field_08;
  msg_fields[8] = robot_info->data_field_09;
  msg_fields[9] = robot_info->data_field_10;
}

UIInfoPanelModule::UIInfoPanelModule(ros::NodeHandle *nh) {
  robot_info_subscriber = nh->subscribe(
      "robot_info", 10, &UIInfoPanelModule::robotInfoCallback, this);
}

UIInfoPanelModule::~UIInfoPanelModule() { robot_info_subscriber.shutdown(); }

int UIInfoPanelModule::render(cv::Mat &frame) {
  //   Create window at (40, 20) with size 250x80 (width x height) and title
  cvui::window(frame, 10, start_render_y_at + 10, 250, 175, "Info");

  for (int x = 0; x < 10; x++) {
    if (msg_fields[x].empty()) {
      msg_fields[x] = "field: empty";
    }
    cvui::text(frame, 10, start_render_y_at + 35 + (15 * x), msg_fields[x]);
  }

  return start_render_y_at + 185;
}
/* END MODULE */

/* MANUAL NAVIGATION MODULE */
UIManualNavigationModule::UIManualNavigationModule(ros::NodeHandle *nh) {
  twist_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

UIManualNavigationModule::~UIManualNavigationModule() {
  twist_publisher.shutdown();
}

int UIManualNavigationModule::render(cv::Mat &frame) {
  if (cvui::button(frame, 80, start_render_y_at + 40, "   Stop  ")) {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
  }

  if (cvui::button(frame, 80, start_render_y_at + 10, " Forward ")) {
    twist_msg.linear.x = twist_msg.linear.x + velocity_step;
  }

  if (cvui::button(frame, 10, start_render_y_at + 40, " Left ")) {
    twist_msg.angular.z = twist_msg.angular.z + velocity_step;
  }

  if (cvui::button(frame, 175, start_render_y_at + 40, " Right ")) {
    twist_msg.angular.z = twist_msg.angular.z - velocity_step;
  }

  if (cvui::button(frame, 80, start_render_y_at + 70, "Backward")) {
    twist_msg.linear.x = twist_msg.linear.x - velocity_step;
  }

  twist_publisher.publish(twist_msg);

  int velocity_window_y = start_render_y_at + 110;
  int velocity_value_y = start_render_y_at + 110 + 25;

  cvui::window(frame, 10, velocity_window_y, 120, 40, "Linear velocity:");
  // Show the current velocity inside the window
  cvui::printf(frame, 25, velocity_value_y, 0.4, 0xff0000, "%.02f m/sec",
               twist_msg.linear.x);

  // Create window at (320 60) with size 120x40 (width x height) and title
  cvui::window(frame, 140, velocity_window_y, 120, 40, "Angular velocity:");
  // Show the current velocity inside the window
  cvui::printf(frame, 155, velocity_value_y, 0.4, 0xff0000, "%.02f rad/sec",
               twist_msg.angular.z);

  return start_render_y_at + 150;
}
/* END MODULE */

/* POSITION MODULE */
void UIPositionModule::robotPositionCallback(
    const nav_msgs::Odometry::ConstPtr &odom_info) {
  positions[0] = odom_info->pose.pose.position.x;
  positions[1] = odom_info->pose.pose.position.y;
  positions[2] = odom_info->pose.pose.position.z;
}

UIPositionModule::UIPositionModule(ros::NodeHandle *nh) {
  robot_position_subscriber =
      nh->subscribe("odom", 10, &UIPositionModule::robotPositionCallback, this);
}

UIPositionModule::~UIPositionModule() { robot_position_subscriber.shutdown(); }

int UIPositionModule::render(cv::Mat &frame) {
  cvui::window(frame, 10, start_render_y_at + 10, 75, 75, "X");
  cvui::window(frame, 95, start_render_y_at + 10, 75, 75, "Y");
  cvui::window(frame, 175, start_render_y_at + 10, 75, 75, "Z");

  cvui::printf(frame, 25, start_render_y_at + 65, .7, 0xffffff, "%.2f",
               positions[0]);

  cvui::printf(frame, 110, start_render_y_at + 65, .7, 0xffffff, "%.2f",
               positions[1]);

  cvui::printf(frame, 200, start_render_y_at + 65, .7, 0xffffff, "%.2f",
               positions[2]);
  return start_render_y_at + 85;
}
/* END MODULE */

/* DISTANCE MODULE */
UIDistanceModule::UIDistanceModule(ros::NodeHandle *nh) {
  distance_service = nh->serviceClient<std_srvs::Trigger>("/get_distance");
}

UIDistanceModule::~UIDistanceModule() { distance_service.shutdown(); }

int UIDistanceModule::render(cv::Mat &frame) {
  if (cvui::button(frame, 10, start_render_y_at + 10, 95, 75, "Get Distance")) {
     if (distance_service.call(trigger)) {
       last_distance = trigger.response.message;
     } else {
       last_distance = "ERROR";
     }
  }
  cvui::window(frame, 115, start_render_y_at + 10, 140, 75,
               "Distance in meters:");
  cvui::text(frame, 190, start_render_y_at + 65, last_distance, .7, 0xffffff);
  return start_render_y_at + 75;
}

/* END MODULE */