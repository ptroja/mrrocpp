/*!
 * \file ecp_g_cvfradia.h
 * \brief Generator responsible for communication with cvFraDIA (testing purposes).
 * - class declaration
 * \author tkornuta
 * \date 17.03.2008
 */

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_cvfradia.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * First step method.. .
 */
bool cvfradia::first_step()
{
	// Set robot parameters.
/*	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.next_interpolation_type
			= EXTERNAL_INTERPOLATION_WITH_FORCE;*/

	communicate_with_edp=false;

	return true;
}

/*!
 * Next step method.
 */
bool cvfradia::next_step()
{
	// Wait for 0.2s.
	usleep(200000);
	// Check trigger.
/*	if (check_and_null_trigger())
	{
		return false;
	}*/
	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

