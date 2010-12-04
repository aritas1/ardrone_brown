#ifndef _ARDRONE_DRIVER_H_
#define _ARDRONE_DRIVER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "androne_sdk.h"

class ARDroneDriver
{
public:
	~ARDroneDriver();

	void updateNavData(navdata_unpacked_t const *const pnd);
	void run();

	static ARDroneDriver &getInstance();

private:
	ARDroneDriver();

	void publish_video();

	ros::NodeHandle node_handle;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber takeoff_sub;
	ros::Subscriber land_sub;
	image_transport::ImageTransport image_transport;
	image_transport::Publisher image_pub;
	ros::Publisher pose_pub;
	ros::Publisher vel_pub;

	int last_frame_id;
	int flying_state;
	static ARDroneDriver *instance;
};

#endif
