#ifndef _TELEOP_TWIST_H_
#define _TELEOP_TWIST_H_

#include "ardrone_sdk.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

extern input_device_t teleop;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
void landCallback(const std_msgs::Empty &msg);
void takeoffCallback(const std_msgs::Empty &msg);

#endif

