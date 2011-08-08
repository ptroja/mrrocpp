#if !defined(MP_R_SMB2_H_)
#define MP_R_SMB2_H_

/*!
 * @file
 * @brief File contains mp robot class declaration for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "mp_r_smb.h"
#include "const_smb2.h"

namespace mrrocpp {
namespace mp {
namespace robot {

/*!
 * @brief SwarmItFix mobile base mp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup smb
 */
class smb2 : public smb
{
public:
	/**
	 * @brief constructor
	 * @param mp_object_l mp task object reference
	 */
	smb2(task::task &mp_object_l);
};

} // namespace robot
} // namespace mp
} // namespace mrrocpp
#endif /*MP_R_SMB2_H_*/
