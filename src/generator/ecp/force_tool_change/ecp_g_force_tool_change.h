#if !defined(_ECP_GEN_FORCE_TOOL_CHANGE_H)
#define _ECP_GEN_FORCE_TOOL_CHANGE_H

/*!
 * @file
 * @brief File contains force_tool_change generator declaration
 *
 * generator commands the new force tool definition to EDP
 * the parameters are: x, y, z position of mass center ant tool mass
 *
 * TODO conditional execution to be implemented
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_force_tool_change.h"

#include "generator/ecp/teach_in/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class force_tool_change : public common::generator::generator
{
protected:
	double tool_parameters[3];
	double weight;
public:
	force_tool_change(common::task::task& _ecp_task);

	void set_tool_parameters(double x, double y, double z, double weight);

	bool first_step();

	// TODO: to be implemented
	void conditional_execution();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
