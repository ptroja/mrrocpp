/**
 * \file	ecp_mp_t_sk_mr.h
 * \brief sk_mr
 * \author yoyek
 * \date	2010
 */

#ifndef __ECP_MP_T_SK_MR_H
#define __ECP_MP_T_SK_MR_H

namespace mrrocpp {
namespace ecp_mp {
namespace task {

/**
 * Used generators.
 */
enum SK_MR_ECP_STATES {
	ECP_GEN_TFF_NOSE_RUN = 0, ECP_GEN_EDGE_FOLLOW_FORCE, ECP_GEN_BIAS_EDP_FORCE
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
