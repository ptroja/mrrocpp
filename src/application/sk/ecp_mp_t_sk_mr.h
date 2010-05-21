/**
 * \file	ecp_mp_t_sk_mr_test.h
 * \brief sk_mr_test
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
enum SK_MR_TEST_ECP_STATES {
	ECP_GEN_TRANSPARENT = 0, ECP_GEN_SLEEP, ECP_GEN_EDGE_FOLLOW
};

} // namespace task
} // namespace ecp_mp
} // namespace mrrocpp

#endif
