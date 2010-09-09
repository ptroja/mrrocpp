#if !defined(MP_R_IRP6OT_M_H_)
#define MP_R_IRP6OT_M_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for IRp6 on track manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6ot_m
 */

#include "base/mp/mp_robot.h"
#include "robot/irp6ot_m/const_irp6ot_m.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief Irp6 on track manipulator mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup irp6ot_m
 */
class irp6ot_m : public robot
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	irp6ot_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_M_H_*/
