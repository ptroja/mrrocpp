#if !defined(MP_R_SHEAD1_H_)
#define MP_R_SHEAD1_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include "mp_r_shead.h"
#include "const_shead1.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix parallel manipulator mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup shead
 */
class shead1 : public shead
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	shead1(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SHEAD1_H_*/
