/*!
 * \file edp_e_smb.h
 * \brief File containing the declaration of edp::smb::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_SMB2_H
#define __EDP_E_SMB2_H

#include "edp_e_smb.h"

namespace mrrocpp {
namespace edp {
namespace smb2 {

/*!
 * \brief class of EDP SwarmItFix mobile base
 *
 * This mobile platform is the base of the SPKM manipulator
 */
class effector : public smb::effector
{
protected:

public:

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell);

};

} // namespace smb2
} // namespace edp
} // namespace mrrocpp


#endif
