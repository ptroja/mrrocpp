#if !defined(_ECP_R_CONVEYOR_H)
#define _ECP_R_CONVEYOR_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */

#include "base/ecp/ecp_robot.h"
#include "robot/conveyor/const_conveyor.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {

/*!
 * @brief Conveyor ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup conveyor
 */
class robot : public common::robot::ecp_robot
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
	robot(common::task::task& _ecp_object);

}; // end: class ecp_conveyor_robot
// ---------------------------------------------------------------

} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp

#endif
