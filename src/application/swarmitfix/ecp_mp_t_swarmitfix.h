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
enum SWARMITFIX_ECP_STATES {
	ECP_GEN_TRANSPARENT = 0,
	ECP_GEN_SMOOTH,
	ECP_GEN_SLEEP,
	ECP_GEN_EPOS,
	ECP_GEN_PIN_LOCK,
	ECP_GEN_PIN_UNLOCK,
	ECP_GEN_PIN_RISE,
	ECP_GEN_PIN_LOWER,
	ECP_GEN_HEAD_SOLDIFY,
	ECP_GEN_HEAD_DESOLDIFY,
	ECP_GEN_VACUUM_ON,
	ECP_GEN_VACUUM_OFF
};

/**
 * Type of the motion in which the smooth generator works (relative or absolute).
 */
enum SMOOTH_MOTION_TYPE {
	RELATIVE, ABSOLUTE
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
