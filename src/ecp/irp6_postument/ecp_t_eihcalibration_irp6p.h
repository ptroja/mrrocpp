//Zadanie kalibracji oko - reka wykorzystujace smooth generator
#if !defined(_ECP_T_EIHCALIBRATION_IRP6P_H)
#define _ECP_T_EIHCALIBRATION_IRP6P_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp_mp/ecp_mp_s_cvfradia.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class eihcalibration: public common::task::task {

	protected:
		common::generator::tff_nose_run* nrg;
		common::generator::smooth* smoothgen;

	public:
		eihcalibration(lib::configurator &_config);
		~eihcalibration();
		void task_initialization(void);
		void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif

