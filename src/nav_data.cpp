#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include "ardrone_sdk.h"

C_RESULT navdata_init(void *private_data)
{
	return C_OK;
}

C_RESULT navdata_process(navdata_unpacked_t const *const pnd)
{
	geometry_msgs::Pose  pose;
	geometry_msgs::Twist vel;

	// TODO: Wrap the battery life in a standard ROS message.
	printf("%d\n", pnd->navdata_demo.vbat_flying_percentage);

	// Convert the Tait-Bryan angles returned by the SDK into a quaternion used
	// by the ROS's TF node.
	double pitch     = pnd->navdata_demo.theta * (1000.0 * M_PI) / 180;
	double yaw       = pnd->navdata_demo.psi   * (1000.0 * M_PI) / 180;
	double roll      = pnd->navdata_demo.phi   * (1000.0 * M_PI) / 180;
	double z         = pnd->navdata_demo.altitude;
	pose.orientation = tf::createQuaternionMsgFromRPY(roll, pitch, yaw);
	pose.position    = tf::Point(0.0, 0.0, z);

	// Wrap the linear velocity returned by the drone in a twist message.
	vel.linear.x  = pnd->navdata_demo.vx;
	vel.linear.y  = pnd->navdata_demo.vy;
	vel.linear.z  = pnd->navdata_demo.vz
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;

	// XXX: Pose and velocity messages need to be published, but we don't have
	// access to AR.Drone ROS node in this function.

	return C_OK;
}

C_RESULT navdata_release(void)
{
	return C_OK;
}
