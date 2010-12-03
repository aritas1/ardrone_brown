#include "teleop_twist.h"

inline float max(float a, float b) { return a > b ? a : b; }
inline float min(float a, float b) { return a < b ? a : b; }

bool is_flying = false;
geometry_msgs::Twist cmd_vel;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
	const float maxHorizontalSpeed = 1.0f; // use 0.1f for testing and 1 for the real thing
	cmd_vel.linear.x  = max(min(-msg->linear.x, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.y  = max(min(-msg->linear.y, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.z  = max(min(msg->linear.z, 1), -1);
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = max(min(-msg->angular.z, 1), -1);
}

void landCallback(const std_msgs::Empty &msg)
{
	is_flying = false;
}

void takeoffCallback(const std_msgs::Empty &msg)
{
	is_flying = true;
}

C_RESULT open_teleop(void)
{
//	ardrone_tool_set_ui_pad_select(1);   //toggles the emergency state
	return C_OK;
}

C_RESULT update_teleop(void)
{
	ardrone_tool_set_ui_pad_start(is_flying); // 1 for take off
//	ardrone_tool_set_ui_pad_select(0);  //clears, or mayby does nothing to, the emergency state... can't really tell

	float left_right = cmd_vel.linear.y;
	float front_back = cmd_vel.linear.x;
	float up_down = cmd_vel.linear.z;
	float turn = cmd_vel.angular.z;

	//printf("LR:%f FB:%f UD:%f TURN:%f\n", left_right, front_back, up_down, turn);
	ardrone_at_set_progress_cmd(1, left_right, front_back, up_down, turn);
	return C_OK;
}

C_RESULT close_teleop(void)
{
	return C_OK;
}

input_device_t teleop = {
	"Teleop",
	open_teleop,
	update_teleop,
	close_teleop
};

