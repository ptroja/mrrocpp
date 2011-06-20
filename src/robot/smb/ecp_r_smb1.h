#if !defined(_ECP_R_SMB1_H)
#define _ECP_R_SMB1_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix mobile base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "const_smb1.h"
#include "ecp_r_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb1 {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup smb
 */
class robot : public smb::robot
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
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

#endif
