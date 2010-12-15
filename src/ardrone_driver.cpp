#include <cmath>
#include <ctime>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_broadcaster.h>
#include "ardrone_driver.h"
#include "teleop_twist.h"
#include "video.h"

////////////////////////////////////////////////////////////////////////////////
// class ARDroneDriver
////////////////////////////////////////////////////////////////////////////////

ARDroneDriver *ARDroneDriver::instance = NULL;

ARDroneDriver::ARDroneDriver()
	: image_transport(node_handle), pose_seq(0u)
{
	cmd_vel_sub = node_handle.subscribe("/cmd_vel", 1, &cmdVelCallback);
	takeoff_sub = node_handle.subscribe("/ardrone/takeoff", 1, &takeoffCallback);
	land_sub = node_handle.subscribe("/ardrone/land", 1, &landCallback);
	image_pub = image_transport.advertise("/ardrone/image_raw", 1);

	battery_pub = node_handle.advertise<std_msgs::UInt32>("/ardrone/battery", 1);
	pose_pub    = node_handle.advertise<geometry_msgs::PoseStamped>("/ardrone/pose", 1);
	imu_pub     = node_handle.advertise<sensor_msgs::Imu>("/ardrone/imu", 1);
	twist_pub   = node_handle.advertise<geometry_msgs::TwistStamped>("/ardrone/twist", 1);
}

ARDroneDriver::~ARDroneDriver()
{
}

void ARDroneDriver::run()
{
	ros::Rate loop_rate(40);

	while (node_handle.ok())
	{
		if (current_frame_id != last_frame_id)
		{
			publish_video();
			last_frame_id = current_frame_id;
		}

		ardrone_tool_update();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ARDroneDriver::publish_video()
{
	sensor_msgs::Image msg;
	msg.width = STREAM_WIDTH;
	msg.height = STREAM_HEIGHT;
	msg.encoding = "rgb8";
	msg.is_bigendian = false;
	msg.step = STREAM_WIDTH*3;
	msg.data.resize(STREAM_WIDTH*STREAM_HEIGHT*3);
	std::copy(buffer, buffer+(STREAM_WIDTH*STREAM_HEIGHT*3), msg.data.begin());
	image_pub.publish(msg);
}

void ARDroneDriver::updateNavData(navdata_unpacked_t const *const pnd)
{
	sensor_msgs::Imu          imu;
	common_msgs::PoseStamped  pose;
	common_msgs::TwistStamped twist;
	float64                   bat_percent;

	imu.header.stamp      = ros::Time::now();
	pose.header.stamp     = ros::Time::now();
	twist.header.stamp    = ros::Time::now();
	imu.header.frame_id   = "map";
	pose.header.frame_id  = "map";
	twist.header.frame_id = "map";

	// Convert the Tait-Bryan angles in millidegrees into a quaternion.
	double pitch     = -pnd->navdata_demo.theta * M_PI / (180 * 1000.0);
	double yaw       = -pnd->navdata_demo.psi   * M_PI / (180 * 1000.0);
	double roll      =  pnd->navdata_demo.phi   * M_PI / (180 * 1000.0);
	imu.orientation  = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	// Convert gyros measurements to rad/s from deg/s.
	imu.angular_velocity.x = pnd->navdata_phys_measures.phys_gyros[GYRO_X] * M_PI / (180 * 1000.0);
	imu.angular_velocity.y = pnd->navdata_phys_measures.phys_gyros[GYRO_Y] * M_PI / (180 * 1000.0);
	imu.angular_velocity.z = pnd->navdata_phys_measures.phys_gyros[GYRO_Y] * M_PI / (180 * 1000.0);
	twist.twist.angular.x = imu.angular_velocity.x;
	twist.twist.angular.y = imu.angular_velocity.y;
	twist.twist.angular.z = imu.angular_velocity.z;

	// Acceleration is specified in milli-G's instead of m/s^2.
	imu.linear_acceleration.x = pnd->navdata_phys_measures.phys_accs[ACC_X] / (1000.0 * 9.80665);
	imu.linear_acceleration.y = pnd->navdata_phys_measures.phys_accs[ACC_Y] / (1000.0 * 9.80665);
	imu.linear_acceleration.z = pnd->navdata_phys_measures.phys_accs[ACC_Z] / (1000.0 * 9.80665);

	// Wrap the linear velocity returned by the drone in a twist message.
	twist.twist.linear.x = pnd->navdata_demo.vx / 1000.0;
	twist.twist.linear.y = pnd->navdata_demo.vy / 1000.0;
	twist.twist.linear.z = pnd->navdata_demo.vz / 1000.0;

	// TODO: Measure the covariance of the IMU.
	imu.orientation_covariance[0]         = -1.0;
	imu.angular_velocity_covariance[0]    = -1.0;
	imu.linear_acceleration_covariance[0] = -1.0;

	// TODO: Broadcast both battery voltage in addition to charge percentage.
	bat_percent = navdata_demo.vbat_flying_percentage;

	// Empirical tests seem to indicate that the documentation is incorrect
	// and altitude is expressed in millimeters instead of centimeters.
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = pnd->navdata_demo.altitude / 1000.0;

	battery_pub.publish(bat_percent);
	imu_pub.publish(imu);
	pose_pub.publish(pose);
	twist_pub.publish(twist)
}

ARDroneDriver &ARDroneDriver::getInstance(void)
{
	if (instance == NULL) {
		instance = new ARDroneDriver;
	}
	return *instance;
}

////////////////////////////////////////////////////////////////////////////////
// custom_main
////////////////////////////////////////////////////////////////////////////////

extern "C" int custom_main(int argc, char** argv)
{
	int res = ardrone_tool_setup_com( NULL );

	if( FAILED(res) )
	{
		printf("Wifi initialization failed. It means either:\n");
		printf("\t* you're not root (it's mandatory because you can set up wifi connection only as root)\n");
		printf("\t* wifi device is not present (on your pc or on your card)\n");
		printf("\t* you set the wrong name for wifi interface (for example rausb0 instead of wlan0) \n");
		printf("\t* ap is not up (reboot card or remove wifi usb dongle)\n");
		printf("\t* wifi device has no antenna\n");
	}
	else
	{
		ardrone_tool_init(argc, argv);
		ros::init(argc, argv, "ardrone_driver");

		ARDroneDriver::getInstance().run();
	}

	return 0;
}

