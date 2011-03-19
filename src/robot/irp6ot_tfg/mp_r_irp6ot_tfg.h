#if !defined(MP_R_IRP6OT_TFG_H_)
#define MP_R_IRP6OT_TFG_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for IRp6 on track two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_tfg
 */

#include "base/mp/mp_robot.h"
#include "robot/irp6ot_tfg/const_irp6ot_tfg.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief Irp6 on track two finger gripper mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup irp6ot_tfg
 */
class irp6ot_tfg : public robot
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	irp6ot_tfg(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_TFG_H_*/

