#include "ardrone_driver.h"
#include "teleop_twist.h"
#include "video.h"

////////////////////////////////////////////////////////////////////////////////
// class ARDroneDriver
////////////////////////////////////////////////////////////////////////////////

ARDroneDriver::ARDroneDriver()
	: image_transport(node_handle), instance(NULL)
{
	cmd_vel_sub = node_handle.subscribe("/cmd_vel", 1, &cmdVelCallback);
	takeoff_sub = node_handle.subscribe("/ardrone/takeoff", 1, &takeoffCallback);
	land_sub = node_handle.subscribe("/ardrone/land", 1, &landCallback);
	image_pub = image_transport.advertise("/ardrone/image_raw", 1);
	pose_pub = node_handle.advertise<geometry_msgs::Pose>("/ardrone/pose", 1);
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

ARDroneDriver &ARDroneDriver::getInstance(void)
{
	if (instance == NULL) {
		instance = new ARDroneDriver;
	}
	return instance;
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

