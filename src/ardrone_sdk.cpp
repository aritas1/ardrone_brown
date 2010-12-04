#include "ardrone_sdk.h"
#include "teleop_twist.h"
#include "nav_data.h"
#include "video.h"

extern "C" {
	C_RESULT ardrone_tool_init_custom(int argc, char **argv)
	{
		ardrone_tool_input_add( &teleop );
		START_THREAD( video_update_thread, 0 );
		return C_OK;
	}

	BEGIN_THREAD_TABLE
		THREAD_TABLE_ENTRY( video_update_thread, 20 )
		THREAD_TABLE_ENTRY( navdata_update, 20 )
		THREAD_TABLE_ENTRY( ATcodec_Commands_Client, 20 )
		THREAD_TABLE_ENTRY( ardrone_control, 20 )
	END_THREAD_TABLE

	BEGIN_NAVDATA_HANDLER_TABLE
		NAVDATA_HANDLER_TABLE_ENTRY(navdata_init, navdata_process, navdata_release, NULL)
	END_NAVDATA_HANDLER_TABLE
}

