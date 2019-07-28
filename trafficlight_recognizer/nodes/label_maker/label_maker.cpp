#include <ros/ros.h>
#include "trafficlight_recognizer/label_maker/label_maker_gui.h"
#include <QApplication>

int main(int argc, char* argv[])
{
  // ROS initialization
  ros::init(argc, argv, "label_maker");

  QApplication application(argc, argv);
  LabelMakerGui window;

  window.show();

  return application.exec();
}
