/**
 * \file	ecp_mp_t_rcsc.h
 * \brief General structure of sensor images.
 * \author yoyek
 * \date	2007
 */

#ifndef __ECP_MP_T_RCSC_H
#define __ECP_MP_T_RCSC_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

/**
 *
 */
enum RCSC_GRIPPER_OP
{
	RCSC_GO_VAR_1, RCSC_GO_VAR_2
};

///**
// * Type of the motion in which the smooth generator works (relative or absolute).
// */
//enum GENERATOR_MOTION_TYPE
//{
//	RELATIVE, ABSOLUTE
//};

/**
 *
 */
enum RCSC_TURN_ANGLES
{
	RCSC_CCL_90, RCSC_CL_0, RCSC_CL_90, RCSC_CL_180
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
