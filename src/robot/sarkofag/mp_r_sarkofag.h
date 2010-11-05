#if !defined(MP_R_SARKOFAG_H_)
#define MP_R_SARKOFAG_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "base/mp/mp_robot.h"
#include "robot/sarkofag/const_sarkofag.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief Sarkofag mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup sarkofag
 */
class sarkofag : public robot
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	sarkofag(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SARKOFAG_H_*/

