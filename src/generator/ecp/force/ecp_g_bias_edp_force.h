#if !defined(_ECP_GEN_BIAS_EDP_FORCE_H)
#define _ECP_GEN_BIAS_EDP_FORCE_H

/*!
 * @file
 * @brief File contains bias_edp_force generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class bias_edp_force : public common::generator::generator
{
public:
	// konstruktor
	bias_edp_force(common::task::task& _ecp_task);

	virtual bool first_step();
	virtual bool next_step();
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
