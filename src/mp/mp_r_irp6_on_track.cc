#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_irp6_on_track.h"

mp_irp6_on_track_robot::mp_irp6_on_track_robot (mp_task* mp_object_l) :
		mp_irp6s_and_conv_robot ( ROBOT_IRP6_ON_TRACK,  "[ecp_irp6_on_track]", mp_object_l)
{}