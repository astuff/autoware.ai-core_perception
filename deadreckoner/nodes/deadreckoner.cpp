/*
 *  Copyright (c) 2017, 2019 Nagoya University, AutonomouStuff
 *  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include "deadreckoner/deadreckoner.h"

DeadRecokner::DeadRecokner() : nh_(), private_nh_("~")
{
  twist_sub_ = nh_.subscribe("current_twist", 10, &DeadRecokner::callbackFromCurrentTwist, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("current_odom", 10);
}

void DeadRecokner::callbackFromCurrentTwist(const geometry_msgs::TwistStampedConstPtr& msg)
{
  // TODO(jwhitleyastuff): calculate odom.pose.pose by accumulating
  nav_msgs::Odometry odom;
  odom.header = msg->header;
  odom.twist.twist = msg->twist;
  odom_pub_.publish(odom);
}
