/*!
 * @file edp_e_spkm2.h
 * @brief File containing the declaration of edp::spkm::effector class.
 *
 * @author Tomasz Winiarski
 * @date 2009
 *
 */

#ifndef __EDP_E_SPKM2_H
#define __EDP_E_SPKM2_H

#include "edp_e_spkm.h"

namespace mrrocpp {
namespace edp {
namespace spkm2 {

// Klasa reprezentujaca robota IRp-6 na postumencie.
/*!
 * @brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector : public spkm::effector
{
private:

protected:

public:

	/*!
	 * @brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell);

};

} // namespace spkm2
} // namespace edp
} // namespace mrrocpp


#endif
