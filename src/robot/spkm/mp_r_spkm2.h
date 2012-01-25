#if !defined(MP_R_SPKM2_H_)
#define MP_R_SPKM2_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "mp_r_spkm.h"
#include "const_spkm2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix parallel manipulator mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spkm
 */
class spkm2 : public spkm
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	spkm2(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SPKM2_H_*/
