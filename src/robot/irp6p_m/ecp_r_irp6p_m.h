#if !defined(_ECP_R_IRP6P_M_H)
#define _ECP_R_IRP6P_M_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "base/ecp/ecp_robot.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "base/kinematics/kinematics_manager.h"

#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {

namespace lib {
class sr_ecp;
class configurator;
}

namespace ecp {
namespace irp6p_m {

/*!
 * @brief IRp6 postument manipulator ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup irp6p_m
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
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

protected:
	virtual void create_kinematic_models_for_given_robot(void);
};

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
