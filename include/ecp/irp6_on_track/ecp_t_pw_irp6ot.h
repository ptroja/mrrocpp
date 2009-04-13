#if !defined(_ECP_T_PW_IRP6OT_H)
#define _ECP_T_PW_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_g_force.h"
#include "common/com_buf.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_g_sleep.h"

#include "ecp/irp6_on_track/ecp_g_pw_kolo.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class pw: public common::task::base  {

	generator::pw_kolo * kolo_gen;

public:
	//Konstruktory.
	pw(lib::configurator &_config);

	//Methods for ECP template to redefine in concrete classes.
	void task_initialization(void);
	void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
