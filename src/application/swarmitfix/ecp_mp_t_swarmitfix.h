/**
* \file	ecp_mp_t_swarmitfix.h
* \brief swarmitfix
* \author yoyek
* \date	2010
*/

#ifndef __ECP_MP_T_SWARMITFIX_H
#define __ECP_MP_T_SWARMITFIX_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

/**
 * Used generators.
 */
enum RCSC_ECP_STATES {ECP_GEN_TRANSPARENT=0, ECP_GEN_SMOOTH  };


/**
 * Type of the motion in which the smooth generator works (relative or absolute).
 */
enum SMOOTH_MOTION_TYPE {RELATIVE, ABSOLUTE};


} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
