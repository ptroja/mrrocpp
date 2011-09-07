#if !defined(MP_R_SHEAD2_H_)
#define MP_R_SHEAD2_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "mp_r_shead.h"
#include "const_shead2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix parallel manipulator mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup shead
 */
class shead2 : public shead
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	shead2(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SHEAD2_H_*/
