/*
 *  Copyright (c) 2017, 2019 Nagoya University, AutonomouStuff
 *  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <ros/ros.h>

#include "deadreckoner/deadreckoner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "deadreckoner");
  DeadRecokner node;
  ros::spin();
  return 0;
}
