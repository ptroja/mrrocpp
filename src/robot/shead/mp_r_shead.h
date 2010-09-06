#if !defined(MP_R_SHEAD_H_)
#define MP_R_SHEAD_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "base/mp/mp_robot.h"
#include "robot/shead/const_shead.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix Head mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup shead
 */
class shead : public robot
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	shead(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SHEAD_H_*/
