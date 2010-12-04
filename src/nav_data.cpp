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
	ARDroneDriver.getInstance().updateNavData(pnd);
	return C_OK;
}

C_RESULT navdata_release(void)
{
	return C_OK;
}
