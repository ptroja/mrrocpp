/*!
 * \file edp_e_shead2.h
 * \brief File containing the declaration of edp::shead2::effector class.
 *
 * \author yoyek
 *
 */

#ifndef __EDP_E_SHEAD2_H
#define __EDP_E_SHEAD2_H

#include "edp_e_shead.h"

namespace mrrocpp {
namespace edp {
namespace shead2 {

/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector : public shead::effector
{
public:
	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell);

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif
