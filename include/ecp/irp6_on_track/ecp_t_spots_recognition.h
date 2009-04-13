/*!
 * \file ecp_t_spotsrecognition.h
 * \brief Declaration of a class responsible
 * for robot moving, used by Piotr Sakowicz.
 * - class declaration
 * \author vented.baffle
 * \date 21.08.2008
 */

#ifndef _ECP_T_SpotsRecognition_H_
#define _ECP_T_SpotsRecognition_H_

#include "ecp/common/ecp_task.h"
#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

//fradia
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_t_cvfradia.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_spots_recognition.h"

#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_t_spots_recognition: public common::task::base
{
	char * trajektoria_poczatkowa;

	protected:

		generator::ecp_spots_generator* generator;
		common::generator::smooth* smooth;
		common::generator::y_nose_run_force* nose;


	public:
		ecp_t_spots_recognition(configurator &_config);
		~ecp_t_spots_recognition();

		void task_initialization(void);
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

