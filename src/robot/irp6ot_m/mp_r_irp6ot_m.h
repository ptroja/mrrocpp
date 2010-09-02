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

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6ot_m : public robot
{
public:
	irp6ot_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6OT_M_H_*/
