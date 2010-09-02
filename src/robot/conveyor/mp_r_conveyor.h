#if !defined(MP_R_CONVEYOR_H_)
#define MP_R_CONVEYOR_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for Conveyor
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup conveyor
 */


#include "base/mp/mp_robot.h"

namespace mrrocpp {
namespace mp {
namespace robot {

class conveyor : public robot
{
public:
	conveyor(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_CONVEYOR_H_*/
