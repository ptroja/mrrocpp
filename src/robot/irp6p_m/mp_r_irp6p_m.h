#if !defined(MP_R_IRP6P_M_H_)
#define MP_R_IRP6P_M_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for IRp6 postument manipulator
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_m
 */

#include "base/mp/mp_r_motor_driven.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class irp6p_m : public motor_driven
{
public:
	irp6p_m(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_IRP6P_M_H_*/
