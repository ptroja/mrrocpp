#if !defined(_ECP_R_SHEAD2_H)
#define _ECP_R_SHEAD2_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "const_shead2.h"
#include "ecp_r_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead2 {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup shead
 */
class robot : public shead::robot
{

public:

	/**
	 * @brief constructor called from UI
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _ecp_object ecp tak object reference
	 */
	robot(common::task::task_base& _ecp_object);

};
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif
