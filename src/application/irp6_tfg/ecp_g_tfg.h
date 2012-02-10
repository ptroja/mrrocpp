// -------------------------------------------------------------------------
//                            generator/ecp_g_force.h dla QNX6
// Deklaracje generatorow dla procesow ECP z wykorzystaniem sily
//
// -------------------------------------------------------------------------

#if !defined(_ECP_GEN_TFG_H)
#define _ECP_GEN_TFG_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "generator/ecp/teach_in/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/irp6_tfg/dp_tfg.h"
#include "ecp_mp_g_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace generator {

/** @addtogroup edge_following
 *
 *  @{
 */

class tfg : public common::generator::generator
{
protected:

	const int step_no;
	lib::irp6_tfg::mp_to_ecp_parameters mp_ecp_command;

public:
	void create_ecp_mp_reply();
	void get_mp_ecp_command();

	// konstruktor
	tfg(common::task::task& _ecp_task, int step);

	virtual bool first_step();

	virtual bool next_step();

};
// end:
///

/** @} */// end of edge_following

}// namespace generator
} // namespace irp6_tfg
} // namespace ecp
} // namespace mrrocpp

#endif
