#if !defined(_ECP_R_IRP6P_TFG_H)
#define _ECP_R_IRP6P_TFG_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "base/ecp/ecp_robot.h"
#include "robot/irp6p_tfg/const_irp6p_tfg.h"

#include "base/kinematics/kinematics_manager.h"

#include "robot/irp6p_tfg/kinematic_model_irp6p_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_tfg {

/*!
 * @brief IRp6 postument gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup irp6p_tfg
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{

protected:

	virtual void create_kinematic_models_for_given_robot(void);

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
};

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
