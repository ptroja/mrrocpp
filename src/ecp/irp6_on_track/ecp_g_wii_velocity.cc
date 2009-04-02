#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_wii_velocity_generator::ecp_wii_velocity_generator (ecp_task& _ecp_task) : ecp_tff_nose_run_generator(_ecp_task)
{
	configure_behaviour(UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION);
}

bool ecp_wii_velocity_generator::next_step()
{
	char buffer[200];
	try
	{
		sensor_m[SENSOR_WIIMOTE]->get_reading();
	}
	catch(...)
	{
	}

	++step_no;

	configure_behaviour(UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION);
	configure_velocity (0.1, 0.0, 0.0, 0.0, 0.0, 0.0);

	return true;
}

