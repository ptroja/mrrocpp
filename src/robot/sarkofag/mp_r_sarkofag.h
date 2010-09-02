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

namespace mrrocpp {
namespace mp {
namespace robot {

class sarkofag : public robot
{
public:
	sarkofag(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SARKOFAG_H_*/

