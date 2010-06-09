/**
 * @file generator/ecp_g_force.cc
 * @brief ECP force generators
 * - class declaration
 * @author yoyek
 * @date 01.01.2002
 *
 * $URL: https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/src/ecp/common/generator/ecp_g_force.cc $
 * $LastChangedRevision: 3339 $
 * $LastChangedDate: 2009-12-23 23:24:07 +0100 (Wed, 23 Dec 2009) $
 * $LastChangedBy: yoyek $
 */

// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzystaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_g_eih_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

eih_nose_run::eih_nose_run(common::task::task& _ecp_task, int step) :
	tff_nose_run(_ecp_task, step) {
	count = 0;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool eih_nose_run::next_step() {
	++count;

	if (count > 25) {// co jakis czas generator sie zatrzymuje
		count = 0;
		return false;
	} else if (pulse_check_activated && check_and_null_trigger()) { // Koniec odcinka
		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu


	// wyrzucanie odczytu sil

	if (force_meassure) {
		lib::Homog_matrix current_frame_wo_offset(
				the_robot->reply_package.arm.pf_def.arm_frame);
		current_frame_wo_offset.remove_translation();

		lib::Ft_v_vector force_torque(
				the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

		std::cout << "force: " << force_torque << std::endl;
	}
	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
