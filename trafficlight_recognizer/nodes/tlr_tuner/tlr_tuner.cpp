#include "trafficlight_recognizer/tlr_tuner/mainwindow.h"
#include "trafficlight_recognizer/tlr_tuner/tuner_body.h"

#include <ros/ros.h>

#include <QApplication>

int main(int argc, char *argv[])
{
  ros::init(argc, argv,"tlr_tuner");

  QApplication a(argc, argv);
  MainWindow w;
  TunerBody tuner;

  w.show();
  tuner.launch();

  return a.exec();
}
